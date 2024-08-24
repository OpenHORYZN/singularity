use core::f32;
use std::time::Duration;

use anyhow::Context;
use argus_common::{LocalPosition, MissionItem, MissionNode, MissionPlan, Waypoint};
use itertools::Itertools;
use px4_msgs::msg::{
    OffboardControlMode, TrajectorySetpoint, VehicleGlobalPosition, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
};
use rclrs::Node;
use tracing::{info, trace};
use uuid::Uuid;

pub type SetpointPair = (OffboardControlMode, TrajectorySetpoint);

use crate::{
    topics::Subscribers,
    trajectory::{
        Constraints3D, StateSnapshot, TrajectoryController, TrajectoryPoll, WaypointStatus,
    },
    util::{
        attitude_to_yaw, default, ApproxEq, LocalPositionFeatures, NodeTimestamp,
        VehicleLocalFeatures, VehicleStatusFeatures, VtolMode, XYZTolerance,
    },
};

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct StatefulMissionPlan {
    pub id: Uuid,
    pub nodes: Vec<StatefulMissionNode>,
}

impl From<MissionPlan> for StatefulMissionPlan {
    fn from(value: MissionPlan) -> Self {
        Self {
            id: value.id,
            nodes: value.nodes.into_iter().map(Into::into).collect_vec(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct StatefulMissionNode {
    pub id: Uuid,
    pub item: StatefulMissionItem,
}

impl From<MissionNode> for StatefulMissionNode {
    fn from(value: MissionNode) -> Self {
        Self {
            id: value.id,
            item: value.item.into(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum StatefulMissionItem {
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

pub struct MissionPlanner {
    plan: StatefulMissionPlan,
    current_step: usize,
    trajectory: TrajectoryController,
    constraints: Constraints3D,
}

impl MissionPlanner {
    pub fn init(plan: MissionPlan, constraints: Constraints3D) -> Result<Self, anyhow::Error> {
        // Validate?
        Ok(Self {
            plan: plan.into(),
            current_step: 0,
            trajectory: TrajectoryController::new(),
            constraints,
        })
    }

    pub fn poll(
        &mut self,
        node: &Node,
        subscribers: &Subscribers,
    ) -> Result<MissionSnapshot, anyhow::Error> {
        let local_pos = subscribers.local_position.current();
        let global_pos = subscribers.global_position.current();
        let local = LocalPosition::from_vehicle(local_pos.to_owned());

        let current_pos = local_pos.position().to_nalgebra().cast();
        let current_vel = local_pos.velocity();
        let current_acc = local_pos.acceleration();

        let current_yaw: f64 = attitude_to_yaw(&subscribers.vehicle_attitude.current()).into();

        let current_state = StateSnapshot {
            position: current_pos,
            velocity: current_vel,
            acceleration: current_acc,
        };

        let plan_len = self.plan.nodes.len();
        let current_step = self.current_step as i32;

        let log_step = |s: &str| info!("[{}/{}] {s}", self.current_step + 1, plan_len);

        let StatefulMissionNode { item: current, .. } = self
            .plan
            .nodes
            .get_mut(self.current_step)
            .context("current step invalid")?;

        let mut wp_status = WaypointStatus::NoProgress;

        let setpoint = match current {
            StatefulMissionItem::Init => MissionSnapshot::Step {
                step: current_step,
                setpoint: self.generate_setpoint(node, GeneratorInput::VelocityZero, subscribers),
                do_land: false,
            },

            StatefulMissionItem::Waypoint(_) => {
                let TrajectoryPoll {
                    snapshot,
                    status,
                    current_target,
                } = self
                    .trajectory
                    .poll(node, current_state, current_yaw)
                    .context("traj missing")?;

                let delta_x = current_target.position.x - local.x;
                let delta_y = current_target.position.y - local.y;

                let yaw = if (delta_x.powi(2) + delta_y.powi(2)).sqrt() < 0.5 {
                    f32::NAN
                } else {
                    f32::atan2(delta_y, delta_x)
                };

                wp_status = status;

                MissionSnapshot::Step {
                    step: current_step,
                    setpoint: self.generate_setpoint(
                        node,
                        GeneratorInput::PosVelAcc { snapshot, yaw },
                        subscribers,
                    ),
                    do_land: false,
                }
            }
            StatefulMissionItem::Takeoff { altitude } => {
                let TrajectoryPoll {
                    snapshot,
                    status,
                    current_target,
                } = self
                    .trajectory
                    .poll(node, current_state, current_yaw)
                    .context("traj missing")?;

                wp_status = status;

                let delta_z = (current_target.position.z - local.z).abs() as f64;

                let yaw = if delta_z >= *altitude / 2.0 {
                    f32::NAN
                } else {
                    if let Some(nt) = self.trajectory.get_next_target() {
                        f32::atan2(nt.position.y - local.y, nt.position.x - local.x)
                    } else {
                        f32::NAN
                    }
                };

                MissionSnapshot::Step {
                    step: current_step,
                    setpoint: self.generate_setpoint(
                        node,
                        GeneratorInput::PosVelAcc { snapshot, yaw },
                        subscribers,
                    ),
                    do_land: false,
                }
            }
            StatefulMissionItem::Delay {
                start_time,
                duration,
            } => {
                let now = node.timestamp() as f64 / 1e6;
                let duration = duration.as_secs_f64();
                start_time.get_or_insert_with(|| {
                    log_step(&format!("(I) Delay: Waiting for {:.02}s", duration));
                    now
                });

                let gen_in = match self.trajectory.poll(node, current_state, current_yaw) {
                    Some(poll) => {
                        let TrajectoryPoll { snapshot, .. } = poll;

                        GeneratorInput::PosVelAcc {
                            snapshot,
                            yaw: f32::NAN,
                        }
                    }
                    None => GeneratorInput::VelocityZero,
                };

                MissionSnapshot::Step {
                    step: current_step,
                    setpoint: self.generate_setpoint(node, gen_in, subscribers),
                    do_land: false,
                }
            }
            StatefulMissionItem::Land { initiated } => {
                let mut do_land = false;
                if !*initiated {
                    log_step("(I) Land: Landing initiated");
                    *initiated = true;
                    do_land = true;
                }

                MissionSnapshot::Step {
                    step: current_step,
                    setpoint: None,
                    do_land,
                }
            }
            StatefulMissionItem::FindSafeSpot => todo!(),
            StatefulMissionItem::Transition => todo!(),
            StatefulMissionItem::PrecLand => todo!(),
            StatefulMissionItem::End => {
                log_step("End: Finished");
                MissionSnapshot::Completed
            }
        };

        self.step(
            node,
            &subscribers.vehicle_status.current(),
            &subscribers.vehicle_land.current(),
            local_pos.clone(),
            global_pos.clone(),
            wp_status,
        )?;
        Ok(setpoint)
    }

    fn step(
        &mut self,
        node: &Node,
        status: &VehicleStatus,
        land: &VehicleLandDetected,
        local_pos: VehicleLocalPosition,
        global_pos: VehicleGlobalPosition,
        wp_status: WaypointStatus,
    ) -> Result<bool, anyhow::Error> {
        let StatefulMissionNode { item: current, .. } = self
            .plan
            .nodes
            .get(self.current_step)
            .context("current step invalid")?;

        let plan_len = self.plan.nodes.len();

        let log_step = |s: &str| info!("[{}/{}] {s}", self.current_step + 1, plan_len);

        match current {
            StatefulMissionItem::Init => {
                if status.arming_state == VehicleStatus::ARMING_STATE_ARMED {
                    log_step("Init: Armed, Starting Trajectory");

                    self.current_step += 1;
                    self.trajectory.build(
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
            StatefulMissionItem::Takeoff { .. } => {
                if wp_status.is_progress() {
                    log_step("Takeoff: Completed");

                    self.current_step += 1;
                    return Ok(true);
                }
                Ok(false)
            }
            StatefulMissionItem::Waypoint(w) => {
                let r = self.trajectory.get_current_target().context("hi")?.position;
                match wp_status {
                    WaypointStatus::Progress => {
                        log_step(&format!("Waypoint: Passed {r} ({w:?})"));

                        self.current_step += 1;
                        Ok(true)
                    }
                    WaypointStatus::Finish => {
                        let current_loc = LocalPosition::from_vehicle(local_pos);
                        if current_loc.approx_eq(&r, XYZTolerance::default()) {
                            log_step(&format!("Waypoint: Reached {r} ({w:?})"));

                            self.current_step += 1;
                            return Ok(true);
                        }
                        Ok(false)
                    }
                    WaypointStatus::NoProgress => Ok(false),
                }
            }
            StatefulMissionItem::Delay {
                duration,
                start_time,
            } => {
                let now = node.timestamp() as f64 / 1e6;
                let duration = duration.as_secs_f64();

                if let Some(st) = start_time {
                    let passed_time = now - *st;
                    if passed_time > duration {
                        log_step(&format!("(T) Delay: {:.02}s elapsed, Proceeding", duration));

                        self.current_step += 1;
                        self.trajectory.build(
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
            StatefulMissionItem::Land { initiated } => {
                if *initiated && land.landed {
                    log_step("(T) Land: Completed");

                    self.current_step += 1;
                    self.trajectory.build(
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
            StatefulMissionItem::FindSafeSpot => todo!(),
            StatefulMissionItem::Transition => todo!(),
            StatefulMissionItem::PrecLand => todo!(),
            StatefulMissionItem::End => Ok(true),
        }
    }

    pub fn generate_setpoint(
        &self,
        node: &Node,
        input: GeneratorInput,
        subscribers: &Subscribers,
    ) -> Option<SetpointPair> {
        match input {
            GeneratorInput::PosVelAcc { snapshot, yaw } => {
                let StateSnapshot {
                    position: final_pos,
                    velocity: final_vel,
                    acceleration: final_acc,
                } = snapshot;

                let enable_full = !self.trajectory.is_paused()
                    && matches!(
                        subscribers.vehicle_status.current().vtol_mode(),
                        VtolMode::Multicopter
                    );

                let offb = OffboardControlMode {
                    timestamp: node.timestamp(),
                    position: true,
                    velocity: enable_full,
                    acceleration: enable_full,
                    ..default()
                };

                let setpoint = TrajectorySetpoint {
                    timestamp: node.timestamp(),
                    position: [final_pos.x as f32, final_pos.y as f32, final_pos.z as f32],
                    velocity: [final_vel.x as f32, final_vel.y as f32, final_vel.z as f32],
                    acceleration: [final_acc.x as f32, final_acc.y as f32, final_acc.z as f32],
                    yawspeed: f32::NAN,
                    yaw,
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
                    yaw: f32::NAN,
                    yawspeed: f32::NAN,
                    ..default()
                };
                Some((offb, setpoint))
            }
        }
    }

    pub fn pause_resume(&mut self, pause: bool, node: &Node) {
        if pause {
            info!("GCS requested mission pause");
            self.trajectory.pause(node);
        } else {
            info!("GCS resumed the mission");
            self.trajectory.resume();
        }
    }
}

pub enum GeneratorInput {
    VelocityZero,
    PosVelAcc { snapshot: StateSnapshot, yaw: f32 },
}

#[derive(Debug, Clone)]
pub enum MissionSnapshot {
    Step {
        step: i32,
        setpoint: Option<SetpointPair>,
        do_land: bool,
    },
    Completed,
}

impl From<MissionItem> for StatefulMissionItem {
    fn from(value: MissionItem) -> Self {
        match value {
            MissionItem::Init => Self::Init,
            MissionItem::Takeoff { altitude } => Self::Takeoff { altitude },
            MissionItem::Waypoint(wp) => Self::Waypoint(wp),
            MissionItem::Delay(duration) => Self::Delay {
                duration,
                start_time: None,
            },
            MissionItem::FindSafeSpot => Self::FindSafeSpot,
            MissionItem::Transition => Self::Transition,
            MissionItem::Land => Self::Land { initiated: false },
            MissionItem::PrecLand => Self::PrecLand,
            MissionItem::End => Self::End,
        }
    }
}
