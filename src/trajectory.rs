use std::{fs::File, io::Write};

use itertools::Itertools;
use nalgebra::Vector3;
use rclrs::Node;
use s_curve::{
    s_curve_generator, Derivative, SCurveConstraints, SCurveInput, SCurveParameters,
    SCurveStartConditions,
};
use tracing::error;
use trajectory::{CubicSpline, Trajectory};
use trajgen::TrajectoryGenerator;

use crate::util::{vector_3d, NodeTimestamp, TimedWaypoint, Waypoint};

pub struct TrajectoryController<P: TrajectoryProvider> {
    duration: f64,
    start_time: Option<f64>,
    provider: P,
    writer: File,
}

impl<P: TrajectoryProvider> TrajectoryController<P> {
    pub fn new_constrained(waypoints: Vec<Waypoint>, max_vel: f32) -> Option<Self> {
        let approx_duration = Self::get_raw_length_m(&waypoints) / max_vel as f64;
        let num_waypoints = waypoints.len();
        let path = waypoints
            .iter()
            .enumerate()
            .map(|(n, w)| {
                let partial = n as f64 / num_waypoints as f64;
                let time_set = partial * approx_duration;

                TimedWaypoint {
                    time: time_set,
                    position: w.position,
                }
            })
            .collect_vec();

        let provider = P::new(dbg!(path))?;

        let mut writer = File::create("log.csv").unwrap();
        writer.write(format!("time,desired_x,desired_y,desired_z,real_x,real_y,real_z,desired_vx,desired_vy,desired_vz\n").as_bytes()).unwrap();

        Some(Self {
            duration: approx_duration,
            start_time: None,
            provider,
            writer,
        })
    }

    pub fn get_corrected_state(
        &mut self,
        node: &Node,
        current_pos: Vector3<f64>,
        current_vel: Vector3<f64>,
        current_acc: Vector3<f64>,
    ) -> Option<TrajectorySnapshot> {
        let now = node.timestamp() as f64 / 1e6;
        let desired = self.get_desired_state(node)?;

        let error_velocity = desired.velocity - current_vel;
        let error_accel = desired.acceleration - current_acc;

        let final_vel = (desired.velocity + error_velocity).cast();
        let final_acc = (desired.acceleration + error_accel).cast();

        self.writer
            .write(
                format!(
                    "{now},{},{},{},{},{},{},{},{},{}\n",
                    desired.position.x,
                    desired.position.y,
                    desired.position.z,
                    current_pos.x,
                    current_pos.y,
                    current_pos.z,
                    desired.velocity.x,
                    desired.velocity.y,
                    desired.velocity.z
                )
                .as_bytes(),
            )
            .unwrap();

        Some(TrajectorySnapshot {
            position: desired.position,
            velocity: final_vel,
            acceleration: final_acc,
        })
    }

    pub fn get_desired_state(&mut self, node: &Node) -> Option<TrajectorySnapshot> {
        let now = node.timestamp() as f64 / 1e6;
        let start_time = *self.start_time.get_or_insert(now);

        //let clamped_duration = f64::min(now - start_time, self.duration);
        let clamped_duration = now - start_time;

        Some(TrajectorySnapshot {
            position: self.provider.get_position(clamped_duration)?,
            velocity: self.provider.get_velocity(clamped_duration)?,
            acceleration: self.provider.get_acceleration(clamped_duration)?,
        })
    }

    pub fn get_raw_length_m(waypoints: &[Waypoint]) -> f64 {
        waypoints
            .windows(2)
            .map(|v| (v[1].position - v[0].position).norm())
            .sum()
    }

    pub fn is_completed(&self, node: &Node) -> bool {
        self.start_time
            .is_some_and(|s| (node.timestamp() as f64 / 1e6) - s > self.duration)
    }
}

#[derive(Debug, Clone)]
pub struct TrajectorySnapshot {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
}

pub trait TrajectoryProvider
where
    Self: Sized,
{
    fn new(waypoints: Vec<TimedWaypoint>) -> Option<Self>;
    fn get_position(&self, t: f64) -> Option<Vector3<f64>>;
    fn get_velocity(&self, t: f64) -> Option<Vector3<f64>>;
    fn get_acceleration(&self, t: f64) -> Option<Vector3<f64>>;
}

impl TrajectoryProvider for CubicSpline<f64> {
    fn new(waypoints: Vec<TimedWaypoint>) -> Option<Self> {
        let (times, points) = waypoints
            .into_iter()
            .map(|wp| (wp.time, vec![wp.position.x, wp.position.y, wp.position.z]))
            .unzip();
        CubicSpline::new(times, points)
    }
    fn get_position(&self, t: f64) -> Option<Vector3<f64>> {
        self.position(t).map(|p| vector_3d(p))
    }

    fn get_velocity(&self, t: f64) -> Option<Vector3<f64>> {
        self.velocity(t).map(|v| vector_3d(v))
    }

    fn get_acceleration(&self, t: f64) -> Option<Vector3<f64>> {
        self.acceleration(t).map(|a| vector_3d(a))
    }
}

impl TrajectoryProvider for TrajectoryGenerator {
    fn new(waypoints: Vec<TimedWaypoint>) -> Option<Self> {
        let waypoints = waypoints
            .into_iter()
            .map(|i| trajgen::Point {
                x: i.position.x as f32,
                y: i.position.y as f32,
                z: i.position.z as f32,
                t: i.time as f32,
            })
            .collect_vec();
        let generator = TrajectoryGenerator::init(trajgen::TrajectoryType::Snap, waypoints)
            .map_err(|e| error!("{e}"))
            .ok()?;
        Some(generator)
    }

    fn get_position(&self, t: f64) -> Option<Vector3<f64>> {
        self.get_position(t as f32)
            .map(|p| Vector3::new(p.x.into(), p.y.into(), p.z.into()))
    }

    fn get_velocity(&self, t: f64) -> Option<Vector3<f64>> {
        self.get_velocity(t as f32)
            .map(|p| Vector3::new(p.x.into(), p.y.into(), p.z.into()))
    }

    fn get_acceleration(&self, t: f64) -> Option<Vector3<f64>> {
        self.get_acceleration(t as f32)
            .map(|p| Vector3::new(p.x.into(), p.y.into(), p.z.into()))
    }
}

pub type SCurveFinal = (SCurveParameters, Box<dyn Fn(f64) -> f64>);

pub struct SCurveAllDeriv {
    pos: SCurveFinal,
    vel: SCurveFinal,
    acc: SCurveFinal,
}

impl SCurveAllDeriv {
    pub fn generate(start: f64, end: f64, max_vel: f64) -> Self {
        let get_curve = |der| {
            s_curve_generator(
                &SCurveInput {
                    constraints: SCurveConstraints {
                        max_acceleration: 1.0,
                        max_jerk: 1.0,
                        max_velocity: max_vel,
                    },
                    start_conditions: SCurveStartConditions {
                        q0: start,
                        q1: end,
                        v0: 0.0,
                        v1: 0.0,
                    },
                },
                der,
            )
        };

        Self {
            pos: get_curve(Derivative::Position),
            vel: get_curve(Derivative::Velocity),
            acc: get_curve(Derivative::Acceleration),
        }
    }

    pub fn get_position(&self, t: f64) -> f64 {
        self.pos.1(t)
    }
    pub fn get_velocity(&self, t: f64) -> f64 {
        self.vel.1(t)
    }
    pub fn get_acceleration(&self, t: f64) -> f64 {
        self.acc.1(t)
    }
}

pub struct SCurve3D {
    x: SCurveAllDeriv,
    y: SCurveAllDeriv,
    z: SCurveAllDeriv,
}

impl SCurve3D {
    pub fn init(start: Vector3<f64>, end: Vector3<f64>, max_vel: Vector3<f64>) -> Self {
        let x_curve = SCurveAllDeriv::generate(start.x, end.x, max_vel.x);
        let y_curve = SCurveAllDeriv::generate(start.y, end.y, max_vel.y);
        let z_curve = SCurveAllDeriv::generate(start.z, end.z, max_vel.z);

        Self {
            x: x_curve,
            y: y_curve,
            z: z_curve,
        }
    }

    pub fn position(&self, t: f64) -> Vector3<f64> {
        Vector3::new(
            self.x.get_position(t),
            self.y.get_position(t),
            self.z.get_position(t),
        )
    }

    pub fn velocity(&self, t: f64) -> Vector3<f64> {
        Vector3::new(
            self.x.get_velocity(t),
            self.y.get_velocity(t),
            self.z.get_velocity(t),
        )
    }
    pub fn acceleration(&self, t: f64) -> Vector3<f64> {
        Vector3::new(
            self.x.get_acceleration(t),
            self.y.get_acceleration(t),
            self.z.get_acceleration(t),
        )
    }
}

impl TrajectoryProvider for SCurve3D {
    fn new(waypoints: Vec<TimedWaypoint>) -> Option<Self> {
        let wp0 = &waypoints[0];
        let wp1 = &waypoints[1];
        Some(Self::init(
            wp0.position,
            wp1.position,
            Vector3::new(10.0, 10.0, 5.0),
        ))
    }

    fn get_position(&self, t: f64) -> Option<Vector3<f64>> {
        Some(self.position(t))
    }

    fn get_velocity(&self, t: f64) -> Option<Vector3<f64>> {
        Some(self.velocity(t))
    }

    fn get_acceleration(&self, t: f64) -> Option<Vector3<f64>> {
        Some(self.acceleration(t))
    }
}

pub fn inject_s_curve(wps: Vec<TimedWaypoint>) -> Vec<TimedWaypoint> {
    let (Some(first), Some(last)) = (wps.first(), wps.last()) else {
        return wps;
    };
    let separators: Vec<Vec<TimedWaypoint>> = wps
        .windows(2)
        .filter(|w| w.contains(first) || w.contains(last))
        .map(|w| {
            let wp_first = &w[0];
            let wp_second = &w[1];

            let dir = wp_second.position - wp_first.position;
            let half = wp_first.position + (dir * 0.5);

            let start_time = wp_first.time;
            let completion_time = wp_second.time;

            vec![
                TimedWaypoint {
                    time: start_time + (completion_time - start_time) * 0.01,
                    position: wp_first.position,
                },
                TimedWaypoint {
                    time: start_time + (completion_time - start_time) / 2.0,
                    position: half,
                },
                TimedWaypoint {
                    time: start_time + (completion_time - start_time) * 0.99,
                    position: wp_second.position,
                },
            ]
        })
        .collect();

    wps.into_iter()
        .map(|i| vec![i])
        .interleave(separators)
        .flatten()
        .collect_vec()
}

#[cfg(test)]
mod test {
    use nalgebra::Vector3;

    use crate::util::TimedWaypoint;

    use super::inject_s_curve;

    #[test]
    fn check_s_curve() {
        let wps = vec![
            TimedWaypoint {
                time: 1.0,
                position: Vector3::new(1.0, 1.0, 2.0),
            },
            TimedWaypoint {
                time: 2.0,
                position: Vector3::new(2.0, 2.0, 4.0),
            },
        ];

        let new_wps = inject_s_curve(wps);
        assert_eq!(5, new_wps.len())
    }
}
