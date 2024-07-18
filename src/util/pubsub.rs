use std::sync::Arc;

use rclrs::{Node, RclrsError, Subscription};
use tokio::sync::watch::{self, Ref};

use rosidl_runtime_rs::Message;
use tracing::trace;

pub struct Subscriber<T: Message> {
    _subscription: Arc<Subscription<T>>,
    recv: watch::Receiver<T>,
}

impl<T: Message> Subscriber<T> {
    pub fn new(node: &Node, topic: &str) -> Result<Self, RclrsError> {
        let (sender, receiver) = watch::channel(T::default());
        let _subscription: Arc<Subscription<T>> =
            node.create_subscription(topic, rclrs::QOS_PROFILE_SENSOR_DATA, move |msg: T| {
                trace!("Received {msg:?}");
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
        trace!("Publishing {msg:?}");
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
