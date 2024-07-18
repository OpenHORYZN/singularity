use anyhow::Context;
use nalgebra::Vector3;
use tracing::{debug, info, Level};

use px4_msgs::msg::{
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition,
    VehicleLocalPosition, VehicleStatus,
};

use trajectory::{Constraints3D, TrajectoryController, TrajectorySnapshot, Waypoint};

pub mod trajectory;
pub mod util;
pub mod visualize;

use util::{
    default, GlobalPosition, LocalPosition, LocalPositionLike, NodeTimestamp, Publisher, Subscriber,
};

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt().with_max_level(Level::INFO).init();

    visualize::init();

    let context = rclrs::Context::new(std::env::args())?;

    let node = rclrs::Node::new(&context, "republisher")?;

    info!("Node is starting up ...");

    let sub_global_position: Subscriber<VehicleGlobalPosition> =
        Subscriber::new(&node, "/fmu/out/vehicle_global_position")?;

    let sub_local_position: Subscriber<VehicleLocalPosition> =
        Subscriber::new(&node, "/fmu/out/vehicle_local_position")?;

    let sub_vehicle_status: Subscriber<VehicleStatus> =
        Subscriber::new(&node, "/fmu/out/vehicle_status")?;

    let pub_offboard: Publisher<OffboardControlMode> =
        Publisher::new(&node, "/fmu/in/offboard_control_mode")?;

    let pub_traj: Publisher<TrajectorySetpoint> =
        Publisher::new(&node, "/fmu/in/trajectory_setpoint")?;

    let pub_command: Publisher<VehicleCommand> = Publisher::new(&node, "/fmu/in/vehicle_command")?;

    info!("Waiting for Position Lock ...");

    while *sub_local_position.current() == VehicleLocalPosition::default()
        || *sub_global_position.current() == VehicleGlobalPosition::default()
    {
        rclrs::spin_once(node.clone(), None)?;
    }

    info!("Node initialized");

    let home_position_global = sub_global_position.current().to_owned();
    let home_gps: GlobalPosition = home_position_global.into();

    info!("Current Position: {}", home_gps);

    let flyoff = GlobalPosition {
        alt: 7.0,
        ..home_gps
    };

    info!("Flyoff: {}", flyoff);

    let target = GlobalPosition {
        lat: 47.3970,
        lon: 8.5461,
        alt: 7.0,
    };

    info!("Target: {}", target);

    info!("Switching to Offboard mode");

    pub_command.send(VehicleCommand {
        timestamp: node.timestamp(),
        param1: 1.0,
        param2: 6.0,
        command: VehicleCommand::VEHICLE_CMD_DO_SET_MODE.into(),
        ..default()
    });

    let publish_snapshot = |snapshot: Option<TrajectorySnapshot>| {
        if let Some(snapshot) = snapshot {
            let TrajectorySnapshot {
                position: final_pos,
                velocity: final_vel,
                acceleration: final_acc,
            } = snapshot;

            pub_offboard.send(OffboardControlMode {
                timestamp: node.timestamp(),
                position: true,
                velocity: true,
                acceleration: true,
                ..default()
            });

            let setpoint = TrajectorySetpoint {
                timestamp: node.timestamp(),
                position: [final_pos.x as f32, final_pos.y as f32, final_pos.z as f32],
                velocity: [final_vel.x as f32, final_vel.y as f32, final_vel.z as f32],
                acceleration: [final_acc.x as f32, final_acc.y as f32, final_acc.z as f32],
                ..default()
            };

            debug!("Setpoint {:?}", setpoint);
            pub_traj.send(setpoint);
        } else {
            pub_offboard.send(OffboardControlMode {
                timestamp: node.timestamp(),
                velocity: true,
                ..default()
            });

            let setpoint = TrajectorySetpoint {
                timestamp: node.timestamp(),
                position: [f32::NAN, f32::NAN, f32::NAN],
                velocity: [0.0, 0.0, 0.0],
                ..default()
            };
            pub_traj.send(setpoint);
        }
    };

    info!("Waiting for Operator to arm ...");

    while sub_vehicle_status.current().arming_state != VehicleStatus::ARMING_STATE_ARMED {
        publish_snapshot(None);
        rclrs::spin_once(node.clone(), None)?;
    }

    let default_constraints = Constraints3D {
        max_velocity: Vector3::new(4.0, 4.0, 3.0),
        max_acceleration: Vector3::repeat(0.6),
        max_jerk: Vector3::repeat(0.4),
    };

    let current_position_local = sub_local_position.current().to_owned();
    let home_local = LocalPosition::project(&home_gps, &current_position_local)?;
    let flyoff_local = LocalPosition::project(&flyoff, &current_position_local)?;
    let target_local = LocalPosition::project(&target, &current_position_local)?;

    let initial_na = home_local.to_nalgebra().cast();
    let flyoff_na = flyoff_local.to_nalgebra().cast();
    let target_na = target_local.to_nalgebra().cast();

    let waypoints = vec![
        Waypoint::new(initial_na),
        Waypoint::new(flyoff_na).with_constraints(Constraints3D {
            max_acceleration: Vector3::new(0.2, 0.2, 0.2),
            ..default_constraints
        }),
        Waypoint::new(target_na),
    ];

    let mut traj_to_target =
        TrajectoryController::new_constrained(waypoints, default_constraints, 1.0)
            .context("Failed to init Trajectory Controller")?;

    info!(
        "Trip Duration to Target {:.02}s",
        traj_to_target.total_duration()
    );

    loop {
        visualize::set_time(node.timestamp() as f64 / 1e6);
        visualize::send_grid();

        let current_position_global = sub_global_position.current().to_owned();
        let current_position_local = sub_local_position.current().to_owned();

        let global: GlobalPosition = current_position_global.into();

        let current_pos = current_position_local.position();
        let current_vel = current_position_local.velocity();
        let current_acc = current_position_local.acceleration();

        let snapshot = traj_to_target.get_corrected_state(
            &node,
            current_pos.to_nalgebra().cast(),
            current_vel,
            current_acc,
        );

        publish_snapshot(snapshot);

        if (global - target).req(0.00005, 0.3) && current_vel.norm() < 0.3 {
            break;
        }

        rclrs::spin_once(node.clone(), None)?;
    }

    info!("Target reached, returning");
    let waypoints = vec![Waypoint::new(target_na), Waypoint::new(flyoff_na)];

    let mut traj_back = TrajectoryController::new_constrained(waypoints, default_constraints, 1.0)
        .context("Failed to init Trajectory Controller")?;

    info!("Trip Duration to Home {:.02}s", traj_back.total_duration());

    loop {
        visualize::set_time(node.timestamp() as f64 / 1e6);
        visualize::send_grid();

        let current_position_global = sub_global_position.current().to_owned();
        let current_position_local = sub_local_position.current().to_owned();

        let global: GlobalPosition = current_position_global.into();

        let current_pos = current_position_local.position();
        let current_vel = current_position_local.velocity();
        let current_acc = current_position_local.acceleration();

        let snapshot = traj_back.get_corrected_state(
            &node,
            current_pos.to_nalgebra().cast(),
            current_vel,
            current_acc,
        );

        publish_snapshot(snapshot);

        if (global - flyoff.into()).req(0.00005, 0.3) && current_vel.norm() < 0.3 {
            break;
        }

        rclrs::spin_once(node.clone(), None)?;
    }

    info!("Finished");

    pub_command.send(VehicleCommand {
        timestamp: node.timestamp(),
        command: VehicleCommand::VEHICLE_CMD_NAV_LAND.into(),
        ..default()
    });

    Ok(())
}
