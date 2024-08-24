use core::f64;
use rclrs::Node;

use crate::{
    trajectory::{
        splines::{s_curve_spline, Constraints3D, Spline},
        LowLevelWaypoint, StateSnapshot,
    },
    util::NodeTimestamp,
};

pub struct TrajectorySimulator {
    start_time: Option<f64>,
    pause_t: Option<f64>,
    splines: Vec<TimedSpline>,
    step: usize,
}

impl TrajectorySimulator {
    pub fn new_constrained(
        waypoints: Vec<LowLevelWaypoint>,
        constraints: Constraints3D,
    ) -> Option<Self> {
        let splines = s_curve_spline(waypoints.clone(), constraints);

        let mut timed_splines = vec![];
        let mut acc = 0.0;

        for spline in splines {
            let duration = spline.curve.duration();
            timed_splines.push(TimedSpline {
                spline,
                start_time: acc,
                end_time: acc + duration,
            });
            acc += duration;
        }

        Some(Self {
            start_time: None,
            pause_t: None,
            splines: timed_splines,
            step: 0,
        })
    }

    pub fn get_state(&mut self, node: &Node, shift: Option<f64>) -> SimulatorOutput {
        let now = (node.timestamp() as f64 / 1e6) + shift.unwrap_or_default();
        let mut just_started = false;
        let mut start_time = *self.start_time.get_or_insert_with(|| {
            just_started = true;
            now
        });

        if let Some(pause) = self.pause_t {
            start_time = now - pause;
            self.start_time = Some(start_time);
        }

        let t = now - start_time;

        let status = self.progress(t, just_started);

        let current_spline = &self.splines[self.step];
        let current_curve = &current_spline.spline.curve;

        let local_t = t - current_spline.start_time;

        SimulatorOutput {
            snapshot: StateSnapshot {
                position: current_curve.position(local_t),
                velocity: current_curve.velocity(local_t),
                acceleration: current_curve.acceleration(local_t),
            },
            status,
        }
    }

    fn progress(&mut self, t: f64, just_started: bool) -> SimulatorStatus {
        let current_spline = &self.splines[self.step];

        if just_started {
            return SimulatorStatus::Started;
        }

        if t > current_spline.end_time {
            let next_step = self.step + 1;
            if next_step < self.splines.len() {
                self.step = next_step;
                return SimulatorStatus::Passed(current_spline.spline.end_pos.id);
            } else {
                return SimulatorStatus::Clamped(current_spline.spline.end_pos.id);
            }
        }

        SimulatorStatus::InTransit
    }

    pub fn pause(&mut self, node: &Node) {
        let now = node.timestamp() as f64 / 1e6;
        if self.pause_t.is_some() {
            return;
        }

        let Some(start) = self.start_time else {
            return;
        };

        self.pause_t = Some(now - start);
    }

    pub fn resume(&mut self) {
        self.pause_t = None;
    }

    pub fn is_paused(&self) -> bool {
        self.pause_t.is_some()
    }
}

pub struct TimedSpline {
    spline: Spline,
    start_time: f64,
    end_time: f64,
}

pub struct SimulatorOutput {
    pub snapshot: StateSnapshot,
    pub status: SimulatorStatus,
}

pub enum SimulatorStatus {
    Started,
    InTransit,
    Passed(usize),
    Clamped(usize),
}
