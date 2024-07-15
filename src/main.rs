use std::{fs::File, io::Write};

use anyhow::Context;
use nalgebra::Vector3;
use tracing::info;

use px4_msgs::msg::{
    OffboardControlMode, TrajectorySetpoint, VehicleGlobalPosition, VehicleLocalPosition,
};
use trajectory::{SCurve3D, TrajectoryController, TrajectorySnapshot};

pub mod geo;
pub mod trajectory;
pub mod util;

use trajgen::TrajectoryGenerator;
use util::{
    default, GlobalPosition, LocalPosition, NodeTimestamp, Publisher, Subscriber, Waypoint,
};

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt().init();
    let context = rclrs::Context::new(std::env::args())?;

    let node = rclrs::Node::new(&context, "republisher")?;

    info!("Node is starting up ...");

    let sub_global_position: Subscriber<VehicleGlobalPosition> =
        Subscriber::new(&node, "/fmu/out/vehicle_global_position")?;

    let sub_local_position: Subscriber<VehicleLocalPosition> =
        Subscriber::new(&node, "/fmu/out/vehicle_local_position")?;

    let pub_offboard: Publisher<OffboardControlMode> =
        Publisher::new(&node, "/fmu/in/offboard_control_mode")?;

    let pub_traj: Publisher<TrajectorySetpoint> =
        Publisher::new(&node, "/fmu/in/trajectory_setpoint")?;

    info!("Waiting for Position Lock ...");

    while *sub_local_position.current() == VehicleLocalPosition::default()
        || *sub_global_position.current() == VehicleGlobalPosition::default()
    {
        rclrs::spin_once(node.clone(), None)?;
    }

    info!("Node initialized, sending setpoints now ...");

    let home_position_global = sub_global_position.current().to_owned();
    let home_gps: GlobalPosition = home_position_global.into();

    let flyoff = GlobalPosition {
        alt: home_gps.alt + 5.0,
        ..home_gps
    };

    let target = GlobalPosition {
        lat: flyoff.lat + 0.0001,
        ..flyoff
    };

    let current_position_local = sub_local_position.current().to_owned();
    let home_local = LocalPosition::project(&home_gps, &current_position_local)?;
    let flyoff_local = LocalPosition::project(&flyoff, &current_position_local)?;
    let target_local = LocalPosition::project(&target, &current_position_local)?;

    let initial_na = home_local.to_nalgebra().cast();
    //let flyoff_na = flyoff_local.to_nalgebra().cast();
    let target_na = target_local.to_nalgebra().cast();

    let waypoints = vec![
        Waypoint {
            position: initial_na,
        },
        // Waypoint {
        //     position: flyoff_na,
        // },
        Waypoint {
            position: target_na,
        },
    ];

    let mut traj_controller: TrajectoryController<SCurve3D> =
        TrajectoryController::new_constrained(waypoints, 2.0)
            .context("Failed to init Trajectory Controller")?;

    loop {
        let _current_position_global = sub_global_position.current().to_owned();
        let current_position_local = sub_local_position.current().to_owned();
        let local: LocalPosition = current_position_local.to_owned().into();

        let current_vel = Vector3::new(
            current_position_local.vx,
            current_position_local.vy,
            current_position_local.vz,
        )
        .cast();

        let current_acc = Vector3::new(
            current_position_local.ax,
            current_position_local.ay,
            current_position_local.az,
        )
        .cast();

        let snapshot = traj_controller.get_corrected_state(
            &node,
            local.to_nalgebra().cast(),
            current_vel,
            current_acc,
        );

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

            info!("Setpoint {:?}", setpoint);
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
            info!("Setpoint {:?}", setpoint);
            pub_traj.send(setpoint);
        }

        rclrs::spin_once(node.clone(), None)?;
    }
}
