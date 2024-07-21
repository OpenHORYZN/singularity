use std::fmt::Debug;

use argus_common::LocalPosition;
use nalgebra::Vector3;
use rclrs::Node;
use splines::Spline;
use tracing::debug;

use crate::{
    util::NodeTimestamp,
    visualize::{self, Color},
};

use self::splines::s_curve_spline;
pub use self::splines::Constraints3D;

pub mod splines;

pub struct TrajectoryController {
    start_time: Option<f64>,
    splines: Vec<TimedSpline>,
    step: usize,
}

impl TrajectoryController {
    pub fn new_constrained(
        waypoints: Vec<LowLevelWaypoint>,
        constraints: Constraints3D,
        trans_vel: f64,
    ) -> Option<Self> {
        let splines = s_curve_spline(waypoints.clone(), constraints, trans_vel);

        let mut timed_splines = vec![];
        let mut acc = 0.0;

        for spline in splines {
            let duration = spline.curve.duration();
            debug!("Curve Duration {:.04}s", duration);
            timed_splines.push(TimedSpline {
                spline,
                start_time: acc,
                end_time: acc + duration,
            });
            acc += duration;
        }

        debug!("Total Duration {:.04}s", acc);

        Some(Self {
            start_time: None,
            splines: timed_splines,
            step: 0,
        })
    }

    pub fn get_corrected_state(
        &mut self,
        node: &Node,
        current_pos: Vector3<f64>,
        current_vel: Vector3<f64>,
        current_acc: Vector3<f64>,
    ) -> TrajectoryOutput {
        let TrajectoryOutput {
            snapshot: desired,
            progress,
        } = self.get_desired_state(node);

        let error_velocity = desired.velocity - current_vel;
        let error_accel = desired.acceleration - current_acc;

        let final_vel = (desired.velocity + error_velocity).cast();
        let final_acc = (desired.acceleration + error_accel).cast();

        visualize::log_xyz("trajectory/desired_velocity", desired.velocity);
        visualize::log_xyz("trajectory/actual_velocity", current_vel);
        visualize::log_xyz("trajectory/desired_accel", desired.acceleration);

        visualize::log_pos(
            "trajectory/desired_position",
            desired.position,
            Color::from_rgb(215, 116, 222),
        );

        visualize::log_pos(
            "trajectory/current_position",
            current_pos,
            Color::from_rgb(100, 227, 200),
        );

        TrajectoryOutput {
            snapshot: TrajectorySnapshot {
                position: desired.position,
                velocity: final_vel,
                acceleration: final_acc,
            },
            progress,
        }
    }

    pub fn get_desired_state(&mut self, node: &Node) -> TrajectoryOutput {
        let now = node.timestamp() as f64 / 1e6;
        let mut just_started = false;
        let start_time = *self.start_time.get_or_insert_with(|| {
            just_started = true;
            now
        });

        let t = now - start_time;

        let progress = self.progress(t, just_started);

        let current_spline = &self.splines[self.step];
        let current_curve = &current_spline.spline.curve;

        let local_t = t - current_spline.start_time;

        TrajectoryOutput {
            snapshot: TrajectorySnapshot {
                position: current_curve.position(local_t),
                velocity: current_curve.velocity(local_t),
                acceleration: current_curve.acceleration(local_t),
            },
            progress,
        }
    }

    pub fn total_duration(&self) -> f64 {
        self.splines.last().map(|s| s.end_time).unwrap_or_default()
    }

    fn progress(&mut self, t: f64, just_started: bool) -> TrajectoryProgress {
        let current_spline = &self.splines[self.step];

        if just_started {
            return TrajectoryProgress::Started(current_spline.spline.start_pos);
        }

        if t > current_spline.end_time {
            let next_step = self.step + 1;
            if next_step < self.splines.len() {
                self.step = next_step;
                return TrajectoryProgress::Passed(current_spline.spline.end_pos);
            } else {
                return TrajectoryProgress::Clamped(current_spline.spline.end_pos);
            }
        } else {
            return TrajectoryProgress::InTransit {
                from: current_spline.spline.start_pos,
                to: current_spline.spline.end_pos,
            };
        }
    }
}

pub struct TimedSpline {
    spline: Spline,
    start_time: f64,
    end_time: f64,
}

pub struct TrajectoryOutput {
    pub snapshot: TrajectorySnapshot,
    pub progress: TrajectoryProgress,
}

pub enum TrajectoryProgress {
    Started(LocalPosition),
    InTransit {
        from: LocalPosition,
        to: LocalPosition,
    },
    Passed(LocalPosition),
    Clamped(LocalPosition),
}

#[derive(Debug, Clone)]
pub struct TrajectorySnapshot {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct LowLevelWaypoint {
    position: Vector3<f64>,
    constraints_to: Option<Constraints3D>,
}

impl LowLevelWaypoint {
    pub fn new(position: Vector3<f64>) -> Self {
        Self {
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
        self.position
    }
}
