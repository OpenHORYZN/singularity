use anyhow::anyhow;
use rclrs::MandatoryParameter;
use std::sync::{mpsc, Arc};
use tokio::{sync::watch, task::spawn_blocking};
use tracing::info;
use util::LocalPositionFeatures;

use px4_msgs::msg::{VehicleGlobalPosition, VehicleLocalPosition};

use argus_common::{ControlRequest, ControlResponse, GlobalPosition, LocalPosition, MissionParams};

pub mod link;
pub mod mission;
pub mod topics;
pub mod trajectory;
pub mod util;

use crate::{
    link::ArgusLink,
    mission::{MissionPlanner, MissionSnapshot},
    topics::{PublisherCommand, Publishers, Subscribers},
    trajectory::Constraints3D,
    util::GlobalPositionFeatures,
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter("singularity=debug")
        .init();

    let context = rclrs::Context::new(std::env::args())?;

    let node = rclrs::Node::new(&context, "singularity")?;

    let machine: MandatoryParameter<Arc<str>> = node
        .declare_parameter("machine")
        .mandatory()
        .map_err(|e| anyhow!("Argument 'machine' missing: {e:?}"))?;

    let machine = machine.get();

    info!("Singularity v0.1 | Node: {machine}");

    let subscribers = Subscribers::init(&node)?;
    let publishers = Publishers::init(&node)?;

    let (mission_in, mission_out) = mpsc::channel();
    let (step_in, step_out) = watch::channel(0);

    let (control_snd_in, control_snd_out) = tokio::sync::mpsc::channel(16);
    let (control_rcv_in, control_rcv_out) = mpsc::channel();

    ArgusLink::init(
        subscribers.clone(),
        mission_in,
        step_out,
        (control_rcv_in, control_snd_out),
        machine.to_string(),
    )
    .await;

    spawn_blocking(move || {
        info!("Waiting for Connection to PX4 ...");

        while *subscribers.local_position.current() == VehicleLocalPosition::default()
            || *subscribers.global_position.current() == VehicleGlobalPosition::default()
        {
            rclrs::spin_once(node.clone(), None)?;
        }

        let initial_local =
            LocalPosition::from_vehicle(subscribers.local_position.current().to_owned());

        let initial_global =
            GlobalPosition::from_vehicle(subscribers.global_position.current().to_owned());

        info!("Received initial Position Frame: {initial_local} {initial_global}");
        info!("Node initialized, Ready for GCS connection");

        loop {
            let _ = step_in.send(-1);
            info!("Waiting for Mission ...");
            let mission = loop {
                if let Ok(m) = mission_out.try_recv() {
                    info!("Received Mission: {m}");
                    break m;
                }

                rclrs::spin_once(node.clone(), None)?;
            };

            let MissionParams {
                target_velocity,
                target_acceleration,
                target_jerk,
                ..
            } = mission.params;

            let default_constraints = Constraints3D {
                max_velocity: target_velocity,
                max_acceleration: target_acceleration,
                max_jerk: target_jerk,
            };

            let home_position_global = subscribers.global_position.current().to_owned();
            let home_gps = GlobalPosition::from_vehicle(home_position_global);

            info!("Current Position: {}", home_gps);

            info!("Switching to Offboard mode");

            publishers.send_command(&node, PublisherCommand::SetModeOffboard);

            info!("Waiting for Operator to arm ...");

            let mut mp = MissionPlanner::init(mission.to_owned(), default_constraints)?;

            loop {
                match mp.poll(&node, &subscribers)? {
                    MissionSnapshot::Step {
                        step,
                        setpoint,
                        do_land,
                    } => {
                        if let Some((spo, spt)) = setpoint {
                            publishers.offboard.send(spo);
                            publishers.trajectory.send(spt);
                        }
                        if do_land {
                            publishers.send_command(&node, PublisherCommand::Land);
                        }
                        if let Ok(d) = control_rcv_out.try_recv() {
                            match d {
                                ControlRequest::FetchMissionPlan => {
                                    let _ = control_snd_in.try_send(
                                        ControlResponse::SendMissionPlan(mission.to_owned()),
                                    );
                                }
                                ControlRequest::PauseResume(pause) => {
                                    mp.pause_resume(pause, &node);
                                    let _ = control_snd_in
                                        .try_send(ControlResponse::PauseResume(pause));
                                }
                            }
                        }
                        let _ = step_in.send(step);
                        rclrs::spin_once(node.clone(), None)?;
                    }
                    MissionSnapshot::Completed => break,
                }
            }
        }
    })
    .await?
}
