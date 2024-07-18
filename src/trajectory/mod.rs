use std::{collections::VecDeque, fmt::Debug};

use nalgebra::Vector3;
use rclrs::Node;
use tracing::debug;

use crate::{
    util::NodeTimestamp,
    visualize::{self, Color},
};

pub use self::splines::Constraints3D;
use self::splines::{s_curve_spline, SCurve3D};

pub mod splines;

pub struct TrajectoryController {
    start_time: Option<f64>,
    spline_stack: VecDeque<(f64, SCurve3D)>,
    total_duration: f64,
}

impl TrajectoryController {
    pub fn new_constrained(
        waypoints: Vec<Waypoint>,
        constraints: Constraints3D,
        trans_vel: f64,
    ) -> Option<Self> {
        let curves = s_curve_spline(waypoints, constraints, trans_vel);

        let mut spline_stack = VecDeque::new();
        let mut acc = 0.0;

        for curve in curves {
            debug!("Curve Duration {:.04}s", curve.duration());
            acc += curve.duration();
            spline_stack.push_back((acc, curve));
        }

        debug!("Total Duration {:.04}s", acc);

        Some(Self {
            start_time: None,
            spline_stack,
            total_duration: acc,
        })
    }

    pub fn get_corrected_state(
        &mut self,
        node: &Node,
        current_pos: Vector3<f64>,
        current_vel: Vector3<f64>,
        current_acc: Vector3<f64>,
    ) -> Option<TrajectorySnapshot> {
        let desired = self.get_desired_state(node)?;

        let error_velocity = desired.velocity - current_vel;
        let error_accel = desired.acceleration - current_acc;

        let final_vel = (desired.velocity + error_velocity).cast();
        let final_acc = (desired.acceleration + error_accel).cast();

        visualize::log_xyz("trajectory/desired_velocity", desired.velocity);
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

        Some(TrajectorySnapshot {
            position: desired.position,
            velocity: final_vel,
            acceleration: final_acc,
        })
    }

    pub fn get_desired_state(&mut self, node: &Node) -> Option<TrajectorySnapshot> {
        let now = node.timestamp() as f64 / 1e6;
        let start_time = *self.start_time.get_or_insert(now);

        let t = now - start_time;

        if self.spline_stack.front().is_some_and(|s| t > s.0) && self.spline_stack.len() > 1 {
            self.spline_stack.pop_front();
        }

        let (end, target_spline) = self.spline_stack.front()?;
        let begin = end - target_spline.duration();
        let t = t - begin;

        Some(TrajectorySnapshot {
            position: target_spline.position(t),
            velocity: target_spline.velocity(t),
            acceleration: target_spline.acceleration(t),
        })
    }

    pub fn total_duration(&self) -> f64 {
        self.total_duration
    }
}
#[derive(Debug, Clone)]
pub struct TrajectorySnapshot {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct Waypoint {
    position: Vector3<f64>,
    constraints_to: Option<Constraints3D>,
}

impl Waypoint {
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
}
