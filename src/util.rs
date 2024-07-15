use std::{ops::Sub, sync::Arc};

use nalgebra::Vector3;
use px4_msgs::msg::{VehicleGlobalPosition, VehicleLocalPosition};
use rclrs::{Node, RclrsError, Subscription};
use thiserror::Error;
use tokio::sync::watch::{self, Ref};

use rosidl_runtime_rs::Message;
use tracing::debug;

use crate::geo::MapProjection;

pub struct Subscriber<T: Message> {
    _subscription: Arc<Subscription<T>>,
    recv: watch::Receiver<T>,
}

impl<T: Message> Subscriber<T> {
    pub fn new(node: &Node, topic: &str) -> Result<Self, RclrsError> {
        let (sender, receiver) = watch::channel(T::default());
        let _subscription: Arc<Subscription<T>> =
            node.create_subscription(topic, rclrs::QOS_PROFILE_SENSOR_DATA, move |msg: T| {
                debug!("Received {msg:?}");
                let _ = sender.send(msg);
            })?;
        Ok(Self {
            _subscription,
            recv: receiver,
        })
    }

    pub fn subscribe(&self) -> watch::Receiver<T> {
        self.recv.clone()
    }

    pub fn current(&self) -> Ref<T> {
        self.recv.borrow()
    }
}

pub struct Publisher<T: Message> {
    publisher: Arc<rclrs::Publisher<T>>,
}

impl<T: Message> Publisher<T> {
    pub fn new(node: &Node, topic: &str) -> Result<Self, RclrsError> {
        let publisher = node.create_publisher(topic, rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self { publisher })
    }

    pub fn send(&self, msg: T) {
        let _ = self.publisher.publish(msg);
    }
}

pub trait NodeTimestamp {
    fn timestamp(&self) -> u64;
}

impl NodeTimestamp for Node {
    fn timestamp(&self) -> u64 {
        self.get_clock().now().nsec as u64 / 1_000
    }
}

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

        return Ok(Self {
            x: local_x,
            y: local_y,
            z: local_z,
        });
    }

    pub fn expand(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    pub fn to_nalgebra(self) -> Vector3<f32> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl From<VehicleLocalPosition> for LocalPosition {
    fn from(value: VehicleLocalPosition) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl Sub for LocalPosition {
    type Output = Vector3<f32>;
    fn sub(self, rhs: Self) -> Self::Output {
        Vector3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
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

#[derive(Debug, Clone, PartialEq)]
pub struct TimedWaypoint {
    pub time: f64,
    pub position: Vector3<f64>,
}

#[derive(Debug, Clone)]
pub struct Waypoint {
    pub position: Vector3<f64>,
}
