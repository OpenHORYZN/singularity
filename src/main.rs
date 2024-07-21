use std::{net::UdpSocket, time::Duration};

use anyhow::anyhow;
use argus_common::{GlobalPosition, MissionNode, Waypoint};
use mission::{MissionPlanner, SetpointPair};
use nalgebra::Vector3;
use postcard::from_bytes;
use tracing::{info, Level};

use px4_msgs::msg::{
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition,
    VehicleLocalPosition, VehicleStatus,
};

pub mod mission;
pub mod trajectory;
pub mod util;
pub mod visualize;

use trajectory::Constraints3D;
use util::{
    default, GlobalPositionFeatures, NodeTimestamp, Publisher, Subscriber, VehicleLocalFeatures,
};

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt().with_max_level(Level::INFO).init();

    visualize::init();

    let context = rclrs::Context::new(std::env::args())?;

    let node = rclrs::Node::new(&context, "republisher")?;

    let udp = UdpSocket::bind("0.0.0.0:4444")?;

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

    let publish_setpoint = |sp: SetpointPair| {
        pub_offboard.send(sp.0);
        pub_traj.send(sp.1);
    };

    let default_constraints = Constraints3D {
        max_velocity: Vector3::new(4.0, 4.0, 3.0),
        max_acceleration: Vector3::repeat(0.6),
        max_jerk: Vector3::repeat(0.4),
    };

    let mut buf = [0u8; 4000];

    let mission = loop {
        if let Ok((addr, msg)) = udp
            .recv_from(&mut buf)
            .map(|(n, addr)| (addr, &buf[..n]))
            .map_err(|e| anyhow!("{e}"))
            .and_then(|(addr, msg)| Ok((addr, from_bytes(msg)?)))
        {
            let msg: Vec<MissionNode> = msg;
            info!("Received mission from {addr}: {msg:?}");
            break msg;
        }
    };

    let _mission = vec![
        MissionNode::Init,
        MissionNode::Takeoff { altitude: 7.0 },
        MissionNode::Waypoint(Waypoint::LocalOffset(Vector3::new(100.0, 10.0, -3.0))),
        MissionNode::Delay(Duration::from_secs(5)),
        MissionNode::Waypoint(Waypoint::GlobalRelativeHeight {
            lat: 47.397971,
            lon: 8.546164,
            height_diff: 10.0,
        }),
        MissionNode::Land,
        MissionNode::End,
    ];

    let home_position_global = sub_global_position.current().to_owned();
    let home_gps = GlobalPosition::from_vehicle(home_position_global);

    info!("Current Position: {}", home_gps);

    info!("Switching to Offboard mode");

    pub_command.send(VehicleCommand {
        timestamp: node.timestamp(),
        param1: 1.0,
        param2: 6.0,
        command: VehicleCommand::VEHICLE_CMD_DO_SET_MODE.into(),
        ..default()
    });

    info!("Waiting for Operator to arm ...");

    let mut mp = MissionPlanner::init(mission, default_constraints)?;

    loop {
        visualize::set_time(node.timestamp() as f64 / 1e6);
        visualize::send_grid();

        let current_position_local = sub_local_position.current().to_owned();
        let current_position_global = sub_global_position.current().to_owned();

        let current_vel = current_position_local.velocity();
        let current_acc = current_position_local.acceleration();

        let sp = mp.step(
            &node,
            &pub_command,
            &sub_vehicle_status,
            current_position_local,
            current_position_global,
            current_vel,
            current_acc,
        )?;

        if let Some(sp) = sp {
            publish_setpoint(sp);
        }

        rclrs::spin_once(node.clone(), None)?;
    }
}
