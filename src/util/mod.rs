use std::{fmt::Debug, ops::Sub};

use argus_common::{GlobalPosition, LocalPosition, Waypoint};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use px4_msgs::msg::{VehicleAttitude, VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus};
use thiserror::Error;
use tracing::debug;

pub mod geo;
pub mod pubsub;

pub use self::{
    geo::MapProjection,
    pubsub::{NodeTimestamp, Publisher, Subscriber},
};

pub fn default<T: Default>() -> T {
    T::default()
}

pub trait Materialize {
    fn materialize(
        &self,
        local_position: &VehicleLocalPosition,
        global_height: f64,
    ) -> LocalPosition;
}

impl Materialize for Waypoint {
    fn materialize(
        &self,
        local_position: &VehicleLocalPosition,
        global_height: f64,
    ) -> LocalPosition {
        match *self {
            Waypoint::LocalOffset(o) => {
                let current = local_position.position();
                let offset: Vector3<f32> = o.cast();
                let offset: LocalPosition = offset.into();
                return offset + current;
            }
            Waypoint::GlobalFixedHeight {
                lat,
                lon,
                alt: height_amsl,
            } => LocalPosition::project(
                &GlobalPosition {
                    lat,
                    lon,
                    alt: height_amsl as f32,
                },
                &local_position,
            )
            .unwrap(),
            Waypoint::GlobalRelativeHeight {
                lat,
                lon,
                height_diff,
            } => LocalPosition::project(
                &GlobalPosition {
                    lat,
                    lon,
                    alt: (global_height + height_diff) as f32,
                },
                &local_position,
            )
            .unwrap(),
        }
    }
}

pub trait LocalPositionFeatures {
    fn project(
        pos: &GlobalPosition,
        current_pos: &VehicleLocalPosition,
    ) -> Result<LocalPosition, PositionConvertError>;
    fn from_vehicle(vehicle: VehicleLocalPosition) -> LocalPosition;
}

impl LocalPositionFeatures for LocalPosition {
    fn project(
        pos: &GlobalPosition,
        current_pos: &VehicleLocalPosition,
    ) -> Result<Self, PositionConvertError> {
        if !current_pos.xy_global || !current_pos.z_global {
            return Err(PositionConvertError::NoGlobalLock);
        }

        let projector = MapProjection::new(current_pos.ref_lat, current_pos.ref_lon);

        let (local_x, local_y) = projector.project(pos.lat, pos.lon);

        let local_z = current_pos.ref_alt - pos.alt;

        let res = Self {
            x: local_x,
            y: local_y,
            z: local_z,
        };

        debug!("Projected {} to {}", pos, res);

        return Ok(res);
    }
    fn from_vehicle(value: VehicleLocalPosition) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

pub trait GlobalPositionFeatures {
    fn from_vehicle(vehicle: VehicleGlobalPosition) -> GlobalPosition;
}

impl GlobalPositionFeatures for GlobalPosition {
    fn from_vehicle(value: VehicleGlobalPosition) -> GlobalPosition {
        Self {
            lat: value.lat,
            lon: value.lon,
            alt: value.alt,
        }
    }
}

#[derive(Debug, Error)]
pub enum PositionConvertError {
    #[error("No Global XY / Z Lock")]
    NoGlobalLock,
}

pub fn windows_mut<T>(slice: &mut [T], size: usize, mut function: impl FnMut(&mut [T])) {
    if slice.len() < size {
        return;
    }
    for start in 0..=(slice.len().saturating_sub(size)) {
        function(&mut slice[start..][..size]);
    }
}

pub trait VehicleLocalFeatures {
    fn position(&self) -> LocalPosition;
    fn acceleration(&self) -> Vector3<f64>;
    fn velocity(&self) -> Vector3<f64>;
}

impl VehicleLocalFeatures for VehicleLocalPosition {
    fn position(&self) -> LocalPosition {
        LocalPosition {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    fn acceleration(&self) -> Vector3<f64> {
        Vector3::new(self.ax, self.ay, self.az).cast()
    }

    fn velocity(&self) -> Vector3<f64> {
        Vector3::new(self.vx, self.vy, self.vz).cast()
    }
}

pub trait VehicleStatusFeatures {
    fn vtol_mode(&self) -> VtolMode;
}

impl VehicleStatusFeatures for VehicleStatus {
    fn vtol_mode(&self) -> VtolMode {
        if self.is_vtol {
            if !self.in_transition_mode && self.vehicle_type == 1 {
                VtolMode::Multicopter
            } else if !self.in_transition_mode {
                VtolMode::FixedWing
            } else if self.in_transition_mode && self.in_transition_to_fw {
                VtolMode::TransitionToFW
            } else {
                VtolMode::TransitionToMC
            }
        } else {
            VtolMode::Multicopter
        }
    }
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum VtolMode {
    Multicopter,
    FixedWing,
    TransitionToMC,
    TransitionToFW,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct XYZTolerance {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for XYZTolerance {
    fn default() -> Self {
        Self {
            x: 0.5,
            y: 0.5,
            z: 0.5,
        }
    }
}

impl XYZTolerance {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

pub trait ApproxEq: Sub + Sized {
    type Tolerance: Default;
    fn approx_eq(&self, other: &Self, tolerance: Self::Tolerance) -> bool;
}

impl ApproxEq for LocalPosition {
    type Tolerance = XYZTolerance;
    fn approx_eq(&self, other: &Self, tolerance: Self::Tolerance) -> bool {
        let abs = (*self - *other).abs();
        abs.x < tolerance.x && abs.y < tolerance.y && abs.z < tolerance.z
    }
}

impl ApproxEq for f64 {
    type Tolerance = f64;

    fn approx_eq(&self, other: &Self, tolerance: Self::Tolerance) -> bool {
        (self - other).abs() < tolerance
    }
}

pub trait MapErr<T> {
    fn emap(self) -> anyhow::Result<T>;
}

impl<T, E: Debug> MapErr<T> for Result<T, E> {
    fn emap(self) -> anyhow::Result<T> {
        self.map_err(|e| anyhow::anyhow!("{e:?}"))
    }
}

pub fn finite(num: f64) -> f64 {
    if num.is_finite() {
        num
    } else {
        0.0
    }
}

pub fn attitude_to_yaw(attitude: &VehicleAttitude) -> f32 {
    let [w, x, y, z] = attitude.q;
    let quat = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
    let (_, _, yaw) = quat.euler_angles();
    yaw
}
