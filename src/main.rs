use std::{
    sync::{mpsc, Arc},
    time::Duration,
};

use argus_common::GlobalPosition;
use futures_util::SinkExt;
use nalgebra::Vector3;
use postcard::from_bytes;
use tokio::{select, sync::watch, task::spawn_blocking};
use tracing::{error, info, Level};
use zenoh::prelude::r#async::*;

use px4_msgs::msg::{VehicleGlobalPosition, VehicleLocalPosition};

pub mod mission;
pub mod topics;
pub mod trajectory;
pub mod util;
pub mod visualize;

use crate::{
    mission::{MissionPlanner, SetpointPair},
    topics::{PublisherCommand, Publishers, Subscribers},
    trajectory::Constraints3D,
    util::{GlobalPositionFeatures, NodeTimestamp},
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(Level::DEBUG)
        .init();

    visualize::init();

    let context = rclrs::Context::new(std::env::args())?;

    let node = rclrs::Node::new(&context, "republisher")?;

    info!("Node is starting up ...");

    let subscribers = Subscribers::init(&node)?;
    let publishers = Publishers::init(&node)?;

    let (control_in, control_out) = mpsc::channel();
    let (ms_in, mut ms_out) = watch::channel(0);

    let subs = subscribers.clone();
    tokio::spawn(async move {
        let session = Arc::new(zenoh::open(config::default()).res().await.unwrap());
        let mut pos_pub = session.declare_publisher("position").res().await.unwrap();
        let mut ms_pub = session
            .declare_publisher("mission/step")
            .res()
            .await
            .unwrap();
        let subscriber = session
            .declare_subscriber("mission/update")
            .reliable()
            .res()
            .await
            .unwrap();

        let pos = subs.global_position.subscribe();

        loop {
            let mut pos = pos.clone();
            select! {
                control = subscriber.recv_async() => {
                    match control {
                        Ok(control) => {
                            match from_bytes(&control.value.payload.contiguous()) {
                                Ok(plan) => {
                                    control_in.send(plan).unwrap();
                                },
                                Err(e) => {
                                    error!("Failed to deserialize {e}");
                                }
                            }
                        },
                        Err(e) => println!("{e}"),
                    }
                },
                Ok(_) = pos.changed() => {
                    let update = GlobalPosition::from_vehicle(pos.borrow().to_owned());
                    let data = postcard::to_allocvec(&update).unwrap();
                    if let Err(e) = pos_pub.send(data).await {
                        error!("{e}")
                    }
                }
                Ok(_) = ms_out.changed() => {
                    let step = *ms_out.borrow();
                    let data = postcard::to_allocvec(&step).unwrap();
                    if let Err(e) = ms_pub.send(data).await {
                        error!("{e}")
                    }

                }
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    });

    spawn_blocking(move || {
        info!("Waiting for Position Lock ...");

        while *subscribers.local_position.current() == VehicleLocalPosition::default()
            || *subscribers.global_position.current() == VehicleGlobalPosition::default()
        {
            rclrs::spin_once(node.clone(), None)?;
        }

        info!("Node initialized");

        let publish_setpoint = |sp: SetpointPair| {
            publishers.offboard.send(sp.0);
            publishers.trajectory.send(sp.1);
        };

        let default_constraints = Constraints3D {
            max_velocity: Vector3::new(4.0, 4.0, 3.0),
            max_acceleration: Vector3::repeat(0.6),
            max_jerk: Vector3::repeat(0.4),
        };

        let mission = loop {
            if let Ok(m) = control_out.try_recv() {
                break m;
            }

            rclrs::spin_once(node.clone(), None)?;
        };

        let home_position_global = subscribers.global_position.current().to_owned();
        let home_gps = GlobalPosition::from_vehicle(home_position_global);

        info!("Current Position: {}", home_gps);

        info!("Switching to Offboard mode");

        publishers.send_command(&node, PublisherCommand::SetModeOffboard);

        info!("Waiting for Operator to arm ...");

        let mut mp = MissionPlanner::init(mission, default_constraints)?;

        loop {
            visualize::set_time(node.timestamp() as f64 / 1e6);
            visualize::send_grid();

            let sp = mp.step(&node, &subscribers, &publishers, &ms_in)?;

            if let Some(sp) = sp {
                publish_setpoint(sp);
            }

            rclrs::spin_once(node.clone(), None)?;
        }
    })
    .await?
}
