use std::sync::Arc;

use px4_msgs::msg::{
    OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand,
    VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus,
};
use rclrs::Node;

use crate::util::{default, NodeTimestamp, Publisher, Subscriber};

pub struct Subscribers {
    pub local_position: Subscriber<VehicleLocalPosition>,
    pub global_position: Subscriber<VehicleGlobalPosition>,
    pub vehicle_status: Subscriber<VehicleStatus>,
    pub vehicle_attitude: Subscriber<VehicleAttitude>,
}

impl Subscribers {
    pub fn init(node: &Node) -> anyhow::Result<Arc<Self>> {
        let global_position = Subscriber::new(&node, "/fmu/out/vehicle_global_position")?;

        let local_position = Subscriber::new(&node, "/fmu/out/vehicle_local_position")?;

        let vehicle_status = Subscriber::new(&node, "/fmu/out/vehicle_status")?;

        let vehicle_attitude = Subscriber::new(&node, "/fmu/out/vehicle_attitude")?;

        Ok(Arc::new(Self {
            local_position,
            global_position,
            vehicle_status,
            vehicle_attitude,
        }))
    }
}

pub struct Publishers {
    pub offboard: Publisher<OffboardControlMode>,
    pub trajectory: Publisher<TrajectorySetpoint>,
    pub command: Publisher<VehicleCommand>,
}

impl Publishers {
    pub fn init(node: &Node) -> anyhow::Result<Arc<Self>> {
        let offboard: Publisher<OffboardControlMode> =
            Publisher::new(&node, "/fmu/in/offboard_control_mode")?;

        let trajectory: Publisher<TrajectorySetpoint> =
            Publisher::new(&node, "/fmu/in/trajectory_setpoint")?;

        let command: Publisher<VehicleCommand> = Publisher::new(&node, "/fmu/in/vehicle_command")?;

        Ok(Arc::new(Self {
            trajectory,
            offboard,
            command,
        }))
    }

    pub fn send_command(&self, node: &Node, command: PublisherCommand) {
        let cmd = match command {
            PublisherCommand::SetModeOffboard => VehicleCommand {
                timestamp: node.timestamp(),
                param1: 1.0,
                param2: 6.0,
                command: VehicleCommand::VEHICLE_CMD_DO_SET_MODE.into(),
                ..default()
            },

            PublisherCommand::Land => VehicleCommand {
                timestamp: node.timestamp(),
                command: VehicleCommand::VEHICLE_CMD_NAV_LAND.into(),
                ..default()
            },
        };

        self.command.send(cmd);
    }
}

pub enum PublisherCommand {
    SetModeOffboard,
    Land,
}
