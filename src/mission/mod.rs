use std::{process::exit, time::Duration};

use anyhow::Context;
use argus_common::{LocalPosition, MissionNode, Waypoint};
use itertools::Itertools;
use nalgebra::Vector3;
use px4_msgs::msg::{
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition,
    VehicleLocalPosition, VehicleStatus,
};
use rclrs::Node;
use solver::build_trajectory;
use tracing::{info, trace};

pub type SetpointPair = (OffboardControlMode, TrajectorySetpoint);

use crate::{
    trajectory::{
        Constraints3D, TrajectoryController, TrajectoryOutput, TrajectoryProgress,
        TrajectorySnapshot,
    },
    util::{
        default, ApproxEq, LocalPositionFeatures, NodeTimestamp, Publisher, Subscriber,
        XYZTolerance,
    },
};

pub mod solver;

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum StatefulMissionNode {
    Init,
    Takeoff {
        altitude: f64,
    },
    Waypoint(Waypoint),
    Delay {
        duration: Duration,
        start_time: Option<f64>,
    },
    FindSafeSpot,
    Transition,
    Land {
        initiated: bool,
    },
    PrecLand,
    End,
}

impl From<MissionNode> for StatefulMissionNode {
    fn from(value: MissionNode) -> Self {
        match value {
            MissionNode::Init => Self::Init,
            MissionNode::Takeoff { altitude } => Self::Takeoff { altitude },
            MissionNode::Waypoint(wp) => Self::Waypoint(wp),
            MissionNode::Delay(duration) => Self::Delay {
                duration,
                start_time: None,
            },
            MissionNode::FindSafeSpot => Self::FindSafeSpot,
            MissionNode::Transition => Self::Transition,
            MissionNode::Land => Self::Land { initiated: false },
            MissionNode::PrecLand => Self::PrecLand,
            MissionNode::End => Self::End,
        }
    }
}

pub struct MissionPlanner {
    plan: Vec<StatefulMissionNode>,
    current_step: usize,
    trajectory: Option<TrajectoryController>,
    constraints: Constraints3D,
}

impl MissionPlanner {
    pub fn init(plan: Vec<MissionNode>, constraints: Constraints3D) -> Result<Self, anyhow::Error> {
        // Validate?
        Ok(Self {
            plan: plan.into_iter().map(Into::into).collect_vec(),
            current_step: 0,
            trajectory: None,
            constraints,
        })
    }

    pub fn step(
        &mut self,
        node: &Node,
        pub_command: &Publisher<VehicleCommand>,
        sub_status: &Subscriber<VehicleStatus>,
        local_pos: VehicleLocalPosition,
        global_pos: VehicleGlobalPosition,
        current_vel: Vector3<f64>,
        current_acc: Vector3<f64>,
    ) -> Result<Option<SetpointPair>, anyhow::Error> {
        let local = LocalPosition::from_vehicle(local_pos.to_owned());

        let plan_len = self.plan.len();

        let mut progress: Option<TrajectoryProgress> = None;

        let current = self
            .plan
            .get_mut(self.current_step)
            .context("current step invalid")?;

        let setpoint = match current {
            StatefulMissionNode::Init => {
                Ok(Self::generate_setpoint(node, GeneratorInput::VelocityZero))
            }
            StatefulMissionNode::Takeoff { .. } | StatefulMissionNode::Waypoint(_) => {
                let controller = self.trajectory.as_mut().context("traj missing")?;

                let TrajectoryOutput {
                    snapshot,
                    progress: prog,
                } = controller.get_corrected_state(
                    node,
                    local.to_nalgebra().cast(),
                    current_vel,
                    current_acc,
                );

                progress = Some(prog);

                Ok(Self::generate_setpoint(
                    node,
                    GeneratorInput::PosVelAcc(snapshot),
                ))
            }
            StatefulMissionNode::Delay {
                start_time,
                duration,
            } => {
                let now = node.timestamp() as f64 / 1e6;
                if start_time.is_none() {
                    info!(
                        "[{}/{}] (I) Delay: Waiting for {:.02}s",
                        self.current_step + 1,
                        plan_len,
                        duration.as_secs_f64()
                    );
                    *start_time = Some(now);
                }
                let controller = self.trajectory.as_mut().context("missing traj")?;
                let TrajectoryOutput { snapshot, .. } = controller.get_corrected_state(
                    node,
                    local.to_nalgebra().cast(),
                    current_vel,
                    current_acc,
                );

                Ok(Self::generate_setpoint(
                    node,
                    GeneratorInput::PosVelAcc(snapshot),
                ))
            }
            StatefulMissionNode::FindSafeSpot => todo!(),
            StatefulMissionNode::Transition => todo!(),
            StatefulMissionNode::Land { initiated } => {
                if !*initiated {
                    info!(
                        "[{}/{}] (I) Land: Landing initiated",
                        self.current_step + 1,
                        plan_len,
                    );
                    pub_command.send(VehicleCommand {
                        timestamp: node.timestamp(),
                        command: VehicleCommand::VEHICLE_CMD_NAV_LAND.into(),
                        ..default()
                    });
                    *initiated = true;
                }

                Ok(Self::generate_setpoint(node, GeneratorInput::Nothing))
            }
            StatefulMissionNode::PrecLand => todo!(),
            StatefulMissionNode::End => {
                info!("[{}/{}] End: Finished", self.current_step + 1, plan_len,);
                exit(0);
            }
        };

        self.try_advance(node, sub_status, local_pos, global_pos, progress)?;
        setpoint
    }

    fn try_advance(
        &mut self,
        node: &Node,
        sub_status: &Subscriber<VehicleStatus>,
        local_pos: VehicleLocalPosition,
        global_pos: VehicleGlobalPosition,
        progress: Option<TrajectoryProgress>,
    ) -> Result<bool, anyhow::Error> {
        let current = self
            .plan
            .get(self.current_step)
            .context("current step invalid")?;
        match current {
            StatefulMissionNode::Init => {
                if sub_status.current().arming_state == VehicleStatus::ARMING_STATE_ARMED {
                    info!(
                        "[{}/{}] Init: Armed, Starting Trajectory",
                        self.current_step + 1,
                        self.plan.len(),
                    );
                    self.current_step += 1;
                    self.trajectory = build_trajectory(
                        local_pos,
                        global_pos,
                        self.current_step,
                        &self.plan,
                        false,
                        self.constraints,
                    )?;
                    return Ok(true);
                }
                Ok(false)
            }
            StatefulMissionNode::Takeoff { .. } => {
                if let Some(progress) = progress {
                    match progress {
                        TrajectoryProgress::Passed(_) | TrajectoryProgress::Clamped(_) => {
                            info!(
                                "[{}/{}] Takeoff: Completed",
                                self.current_step + 1,
                                self.plan.len(),
                            );
                            self.current_step += 1;
                            return Ok(true);
                        }
                        _ => (),
                    };
                }
                Ok(false)
            }
            StatefulMissionNode::Waypoint(w) => {
                if let Some(progress) = progress {
                    match progress {
                        TrajectoryProgress::Passed(r) => {
                            info!(
                                "[{}/{}] Waypoint: Passed {} ({:?})",
                                self.current_step + 1,
                                self.plan.len(),
                                r,
                                w
                            );
                            self.current_step += 1;
                            return Ok(true);
                        }
                        TrajectoryProgress::Clamped(r) => {
                            let current_loc = LocalPosition::from_vehicle(local_pos);
                            if current_loc.approx_eq(&r, XYZTolerance::default()) {
                                info!(
                                    "[{}/{}] Waypoint: Reached {} ({:?})",
                                    self.current_step + 1,
                                    self.plan.len(),
                                    r,
                                    w
                                );
                                self.current_step += 1;
                                return Ok(true);
                            }
                        }
                        _ => (),
                    };
                }
                Ok(false)
            }
            StatefulMissionNode::Delay {
                duration,
                start_time,
            } => {
                let now = node.timestamp() as f64 / 1e6;
                if let Some(st) = start_time {
                    let passed_time = now - *st;
                    if passed_time > duration.as_secs_f64() {
                        info!(
                            "[{}/{}] (T) Delay: {:.02}s elapsed, Proceeding",
                            self.current_step + 1,
                            self.plan.len(),
                            duration.as_secs_f64()
                        );
                        self.current_step += 1;
                        self.trajectory = build_trajectory(
                            local_pos,
                            global_pos,
                            self.current_step,
                            &self.plan,
                            true,
                            self.constraints,
                        )?;
                        return Ok(true);
                    }
                }
                return Ok(false);
            }
            StatefulMissionNode::FindSafeSpot => todo!(),
            StatefulMissionNode::Transition => todo!(),
            StatefulMissionNode::Land { initiated } => {
                if *initiated
                    && sub_status.current().arming_state == VehicleStatus::ARMING_STATE_DISARMED
                {
                    info!(
                        "[{}/{}] (T) Land: Completed",
                        self.current_step + 1,
                        self.plan.len(),
                    );
                    self.current_step += 1;
                    return Ok(true);
                }
                return Ok(false);
            }
            StatefulMissionNode::PrecLand => todo!(),
            StatefulMissionNode::End => return Ok(true),
        }
    }

    pub fn generate_setpoint(node: &Node, input: GeneratorInput) -> Option<SetpointPair> {
        match input {
            GeneratorInput::PosVelAcc(snapshot) => {
                let TrajectorySnapshot {
                    position: final_pos,
                    velocity: final_vel,
                    acceleration: final_acc,
                } = snapshot;

                let offb = OffboardControlMode {
                    timestamp: node.timestamp(),
                    position: true,
                    velocity: true,
                    acceleration: true,
                    ..default()
                };

                let setpoint = TrajectorySetpoint {
                    timestamp: node.timestamp(),
                    position: [final_pos.x as f32, final_pos.y as f32, final_pos.z as f32],
                    velocity: [final_vel.x as f32, final_vel.y as f32, final_vel.z as f32],
                    acceleration: [final_acc.x as f32, final_acc.y as f32, final_acc.z as f32],
                    ..default()
                };

                trace!("Setpoint {:?}", setpoint);
                Some((offb, setpoint))
            }
            GeneratorInput::VelocityZero => {
                let offb = OffboardControlMode {
                    timestamp: node.timestamp(),
                    velocity: true,
                    ..default()
                };

                let setpoint = TrajectorySetpoint {
                    timestamp: node.timestamp(),
                    position: [f32::NAN, f32::NAN, f32::NAN],
                    velocity: [0.0, 0.0, 0.0],
                    ..default()
                };
                Some((offb, setpoint))
            }
            GeneratorInput::Nothing => None,
        }
    }
}

pub enum GeneratorInput {
    Nothing,
    VelocityZero,
    PosVelAcc(TrajectorySnapshot),
}
