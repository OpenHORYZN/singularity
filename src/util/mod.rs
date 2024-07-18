use std::{fmt::Display, ops::Sub};

use nalgebra::Vector3;
use px4_msgs::msg::{VehicleGlobalPosition, VehicleLocalPosition};
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

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
pub struct LocalPosition {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl LocalPosition {
    pub fn project(
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

    pub fn expand(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    pub fn to_nalgebra(self) -> Vector3<f32> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl Sub for LocalPosition {
    type Output = Vector3<f32>;
    fn sub(self, rhs: Self) -> Self::Output {
        Vector3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Display for LocalPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{:.03}, {:.03}, {:.03}]", self.x, self.y, self.z)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
pub struct GlobalPosition {
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
}

impl From<VehicleGlobalPosition> for GlobalPosition {
    fn from(value: VehicleGlobalPosition) -> Self {
        Self {
            lat: value.lat,
            lon: value.lon,
            alt: value.alt,
        }
    }
}

impl Display for GlobalPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({:.06}, {:.06}, {:.06})", self.lat, self.lon, self.alt)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd)]
pub struct GlobalDifference {
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
}

impl GlobalDifference {
    pub fn req(&self, tol_hh: f64, tol_v: f32) -> bool {
        return self.lat.abs() < tol_hh && self.lon.abs() < tol_hh && self.alt.abs() < tol_v;
    }
}

impl Sub for GlobalPosition {
    type Output = GlobalDifference;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::Output {
            lat: self.lat - rhs.lat,
            lon: self.lon - rhs.lon,
            alt: self.alt - rhs.alt,
        }
    }
}

pub fn vector_3d(vec: Vec<f64>) -> Vector3<f64> {
    Vector3::new(vec[0], vec[1], vec[2])
}

#[derive(Debug, Error)]
pub enum PositionConvertError {
    #[error("No Global XY / Z Lock")]
    NoGlobalLock,
}

pub fn windows_mut<T, F>(slice: &mut [T], size: usize, mut function: F)
where
    F: FnMut(&mut [T]),
{
    if slice.len() < size {
        return;
    }
    for start in 0..=(slice.len().saturating_sub(size)) {
        function(&mut slice[start..][..size]);
    }
}

pub trait LocalPositionLike {
    fn position(&self) -> LocalPosition;
    fn acceleration(&self) -> Vector3<f64>;
    fn velocity(&self) -> Vector3<f64>;
}

impl LocalPositionLike for VehicleLocalPosition {
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
