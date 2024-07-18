use std::fmt::Debug;

use itertools::Itertools;
use nalgebra::Vector3;
use s_curve::{
    s_curve_generator, Derivative, SCurveConstraints, SCurveInput, SCurveParameters,
    SCurveStartConditions,
};

use crate::util::windows_mut;

use super::Waypoint;

pub fn s_curve_spline(
    waypoints: Vec<Waypoint>,
    constraints: Constraints3D,
    trans_vel: f64,
) -> Vec<SCurve3D> {
    let mut path = waypoints
        .windows(2)
        .map(|w| {
            let (primary, secondary) = (&w[0], &w[1]);

            (
                StartConditions3D {
                    start_pos: primary.position,
                    end_pos: secondary.position,
                    start_vel: Vector3::zeros(),
                    end_vel: Vector3::zeros(),
                },
                secondary.constraints_to.clone(),
            )
        })
        .collect_vec();

    windows_mut(&mut path, 2, |slices| {
        let trans_vel_v = (slices[1].0.end_pos - slices[1].0.start_pos)
            .normalize()
            .component_mul(
                &(slices[0].0.end_pos - slices[0].0.start_pos)
                    .normalize()
                    .abs(),
            )
            * trans_vel;
        slices[0].0.end_vel = trans_vel_v;
        slices[1].0.start_vel = trans_vel_v;
    });

    let spline = path
        .into_iter()
        .map(|sc| SCurve3D::init(sc.0, sc.1.unwrap_or(constraints)))
        .collect_vec();

    spline
}

pub type SCurveFinal = (SCurveParameters, Box<dyn Fn(f64) -> f64>);

pub struct SCurveAllDeriv {
    pos: SCurveFinal,
    vel: SCurveFinal,
    acc: SCurveFinal,
}

impl Debug for SCurveAllDeriv {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SCurveAllDeriv")
            .field("pos", &self.pos.0)
            .finish()
    }
}

impl SCurveAllDeriv {
    pub fn generate(constraints: SCurveConstraints, start_cond: SCurveStartConditions) -> Self {
        let input = SCurveInput {
            constraints,
            start_conditions: start_cond,
        };
        let get_curve = |der| s_curve_generator(&input, der);

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

    pub fn get_duration(&self) -> f64 {
        self.pos.0.time_intervals.total_duration()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct StartConditions3D {
    start_pos: Vector3<f64>,
    end_pos: Vector3<f64>,
    start_vel: Vector3<f64>,
    end_vel: Vector3<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct Constraints3D {
    pub max_velocity: Vector3<f64>,
    pub max_acceleration: Vector3<f64>,
    pub max_jerk: Vector3<f64>,
}

#[derive(Debug)]
pub struct SCurve3D {
    x: SCurveAllDeriv,
    y: SCurveAllDeriv,
    z: SCurveAllDeriv,
}

impl SCurve3D {
    pub fn init(start_conditions: StartConditions3D, constraints: Constraints3D) -> Self {
        let get_constraints = |n| SCurveConstraints {
            max_acceleration: constraints.max_acceleration[n],
            max_jerk: constraints.max_jerk[n],
            max_velocity: constraints.max_velocity[n],
        };

        let get_start_cond = |n| SCurveStartConditions {
            q0: start_conditions.start_pos[n],
            q1: start_conditions.end_pos[n],
            v0: start_conditions.start_vel[n],
            v1: start_conditions.end_vel[n],
        };

        let x_curve = SCurveAllDeriv::generate(get_constraints(0), get_start_cond(0));
        let y_curve = SCurveAllDeriv::generate(get_constraints(1), get_start_cond(1));
        let z_curve = SCurveAllDeriv::generate(get_constraints(2), get_start_cond(2));

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

    pub fn duration(&self) -> f64 {
        let fin = |n: f64| if n.is_finite() { n } else { 0.0 };
        0f64.max(fin(self.x.get_duration()))
            .max(fin(self.y.get_duration()))
            .max(fin(self.z.get_duration()))
    }
}
