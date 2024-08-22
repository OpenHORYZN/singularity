use core::f64;
use std::{collections::HashSet, fmt::Debug};

use anyhow::Context;
use argus_common::{GlobalPosition, LocalPosition, Waypoint};
use nalgebra::Vector3;
use px4_msgs::msg::{VehicleGlobalPosition, VehicleLocalPosition};
use rclrs::Node;
use tracing::{debug, error};

use crate::{
    mission::{StatefulMissionItem, StatefulMissionPlan},
    trajectory::simulator::{SimulatorOutput, SimulatorStatus, TrajectorySimulator},
    util::{ApproxEq, LocalPositionFeatures, Materialize, VehicleLocalFeatures},
};

pub use crate::trajectory::splines::Constraints3D;

pub mod simulator;
pub mod splines;

pub struct TrajectoryController {
    sim: Option<TrajectorySimulator>,
    completions: HashSet<usize>,
    waypoints: Vec<LowLevelWaypoint>,
    step: usize,
    manual_pause: bool,
    rotate_first: bool,
}

impl TrajectoryController {
    pub fn new() -> Self {
        Self {
            sim: None,
            completions: HashSet::new(),
            waypoints: vec![],
            step: 0,
            manual_pause: false,
            rotate_first: false,
        }
    }

    pub fn poll(
        &mut self,
        node: &Node,
        current_state: StateSnapshot,
        current_yaw: f64,
    ) -> Option<TrajectoryPoll> {
        let sim = self.sim.as_mut()?;

        let SimulatorOutput {
            snapshot: desired,
            status,
        } = sim.get_state(node);

        if self.rotate_first {
            sim.pause(node);
            debug!("Waiting until Yaw is headed to next Waypoint ...");
            self.rotate_first = false;
        }

        let status = match status {
            SimulatorStatus::Passed(id) => {
                sim.pause(node);
                debug!("Waiting until Yaw is headed to next Waypoint ...");
                self.step += 1;
                if self.completions.contains(&id) {
                    WaypointStatus::Progress
                } else {
                    WaypointStatus::NoProgress
                }
            }
            SimulatorStatus::Clamped(id) => {
                if self.step + 1 < self.waypoints.len() {
                    self.step += 1;
                }
                if self.completions.contains(&id) {
                    WaypointStatus::Finish
                } else {
                    WaypointStatus::NoProgress
                }
            }
            SimulatorStatus::Started => {
                self.step += 1;
                WaypointStatus::NoProgress
            }
            _ => WaypointStatus::NoProgress,
        };

        if sim.is_paused() {
            let Some(tgt) = self.waypoints.get(self.step) else {
                error!("paused but no next target?");
                return None;
            };

            let tgt = tgt.position();

            let delta_x = tgt.x - current_state.position.x;
            let delta_y = tgt.y - current_state.position.y;

            let desired_yaw = f64::atan2(delta_y, delta_x);

            if current_yaw.approx_eq(&desired_yaw, 0.001) && !self.manual_pause {
                debug!("Yaw rotation complete, resuming");
                sim.resume();
            }
        }

        let error_velocity = desired.velocity - current_state.velocity;
        let error_accel = desired.acceleration - current_state.acceleration;

        let final_vel = (desired.velocity + error_velocity).cast();
        let final_acc = (desired.acceleration + error_accel).cast();

        Some(TrajectoryPoll {
            snapshot: StateSnapshot {
                position: desired.position,
                velocity: final_vel,
                acceleration: final_acc,
            },
            status,
            current_target: self.waypoints[self.step].clone(),
        })
    }

    pub fn build(
        &mut self,
        local_position: VehicleLocalPosition,
        global_position: VehicleGlobalPosition,
        current_step: usize,
        plan: &StatefulMissionPlan,
        include_current_pos: bool,
        constraints: Constraints3D,
    ) -> anyhow::Result<()> {
        let forward = plan
            .nodes
            .get(current_step..)
            .context("out of bounds")?
            .iter()
            .map(|i| &i.item);

        let mut waypoint_stack = vec![];

        let mut id = 0;

        self.completions.clear();
        self.rotate_first = include_current_pos;
        self.step = 0;

        if include_current_pos {
            waypoint_stack.push(LowLevelWaypoint::new(id, local_position.position()));
            id += 1;
        }

        for node in forward {
            let last_wp = waypoint_stack.last().cloned();
            let mut push_wp = |pos, constr| {
                let now_id = id;
                let wp = LowLevelWaypoint::new(now_id, pos);
                if let Some(c) = constr {
                    waypoint_stack.push(wp.with_constraints(c));
                } else {
                    waypoint_stack.push(wp);
                }
                id += 1;
                return now_id;
            };
            match node {
                StatefulMissionItem::Takeoff { altitude } => {
                    let current_wp = Waypoint::LocalOffset(Vector3::new(0.0, 0.0, 0.0));
                    let takeoff_wp = Waypoint::LocalOffset(Vector3::new(0.0, 0.0, -altitude));
                    let local_current =
                        current_wp.materialize(&local_position, global_position.alt.into());
                    let local_takeoff =
                        takeoff_wp.materialize(&local_position, global_position.alt.into());
                    push_wp(local_current, None);
                    let takeoff_completed_id = push_wp(
                        local_takeoff,
                        Some(Constraints3D {
                            max_acceleration: Vector3::new(0.2, 0.2, 0.2),
                            ..constraints
                        }),
                    );
                    self.completions.insert(takeoff_completed_id);
                }
                StatefulMissionItem::Waypoint(wp) => {
                    let local = if let Some(lp) = last_wp {
                        match wp {
                            Waypoint::LocalOffset(o) => (lp.position() + o).cast().into(),
                            Waypoint::GlobalFixedHeight { lat, lon, alt } => {
                                LocalPosition::project(
                                    &GlobalPosition {
                                        lat: *lat,
                                        lon: *lon,
                                        alt: *alt as f32,
                                    },
                                    &local_position,
                                )
                                .unwrap()
                            }
                            Waypoint::GlobalRelativeHeight {
                                lat,
                                lon,
                                height_diff,
                            } => {
                                let mut init = LocalPosition::project(
                                    &GlobalPosition {
                                        lat: *lat,
                                        lon: *lon,
                                        alt: global_position.alt,
                                    },
                                    &local_position,
                                )
                                .unwrap();
                                init.z = (lp.position().z - *height_diff) as f32;
                                init
                            }
                        }
                    } else {
                        wp.materialize(&local_position, 0.0)
                    };

                    let wp_id = push_wp(local, None);
                    self.completions.insert(wp_id);
                }
                StatefulMissionItem::Transition => (),
                _ => break,
            }
        }

        if waypoint_stack.len() < 2 {
            return Ok(());
        }

        let sim = TrajectorySimulator::new_constrained(waypoint_stack.clone(), constraints)
            .context("could not create traj c")?;

        self.sim = Some(sim);
        self.waypoints = waypoint_stack;

        Ok(())
    }

    pub fn get_current_target(&self) -> Option<&LowLevelWaypoint> {
        self.waypoints.get(self.step)
    }

    pub fn get_next_target(&self) -> Option<&LowLevelWaypoint> {
        self.waypoints.get(self.step + 1)
    }

    pub fn pause(&mut self, node: &Node) {
        self.sim.as_mut().map(|s| {
            s.pause(node);
            self.manual_pause = true
        });
    }

    pub fn resume(&mut self) {
        self.sim.as_mut().map(|s| {
            s.resume();
            self.manual_pause = false
        });
    }

    pub fn is_paused(&self) -> bool {
        self.sim.as_ref().is_some_and(|s| s.is_paused())
    }
}

pub struct TrajectoryPoll {
    pub snapshot: StateSnapshot,
    pub current_target: LowLevelWaypoint,
    pub status: WaypointStatus,
}

pub enum WaypointStatus {
    NoProgress,
    Progress,
    Finish,
}

impl WaypointStatus {
    pub fn is_progress(&self) -> bool {
        matches!(self, WaypointStatus::Progress | WaypointStatus::Finish)
    }
}

#[derive(Debug, Clone)]
pub struct StateSnapshot {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct LowLevelWaypoint {
    pub id: usize,
    pub position: LocalPosition,
    pub constraints_to: Option<Constraints3D>,
}

impl LowLevelWaypoint {
    pub fn new(id: usize, position: LocalPosition) -> Self {
        Self {
            id,
            position,
            constraints_to: None,
        }
    }

    pub fn with_constraints(self, constraints: Constraints3D) -> Self {
        Self {
            constraints_to: Some(constraints),
            ..self
        }
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position.to_nalgebra().cast()
    }
}
