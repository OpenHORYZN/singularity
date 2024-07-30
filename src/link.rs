use std::{
    error::Error,
    sync::{mpsc, Arc},
    time::Duration,
};

use argus_common::{ControlRequest, ControlResponse, GlobalPosition, MissionNode};
use futures_util::SinkExt;
use postcard::from_bytes;
use tokio::{select, sync::watch};
use tracing::error;
use zenoh::prelude::r#async::*;

use crate::{topics::Subscribers, util::GlobalPositionFeatures};

pub type ZenohError = Box<dyn Error + Send + Sync>;

pub struct ArgusLink;

impl ArgusLink {
    pub async fn init(
        subs: Arc<Subscribers>,
        mission_in: mpsc::Sender<Vec<MissionNode>>,
        mut step_out: watch::Receiver<i32>,
        mut control_link: (
            mpsc::Sender<ControlRequest>,
            tokio::sync::mpsc::Receiver<ControlResponse>,
        ),
        machine: String,
    ) -> Self {
        tokio::spawn(async move {
            let t = |o: &str| format!("{machine}/{o}");
            let session = Arc::new(zenoh::open(config::default()).res().await?);
            let mut pos_pub = session.declare_publisher(t("position")).res().await?;
            let mut step_pub = session.declare_publisher(t("mission/step")).res().await?;
            let mut control_pub = session.declare_publisher(t("control/out")).res().await?;

            let mission_sub = session
                .declare_subscriber(t("mission/update"))
                .reliable()
                .res()
                .await?;

            let control_sub = session
                .declare_subscriber(t("control/in"))
                .reliable()
                .res()
                .await?;

            let pos = subs.global_position.subscribe();

            loop {
                let mut pos = pos.clone();
                select! {
                    mission = mission_sub.recv_async() => {
                        match mission {
                            Ok(mission) => {
                                match from_bytes(&mission.value.payload.contiguous()) {
                                    Ok(plan) => {
                                        mission_in.send(plan).unwrap();
                                    },
                                    Err(e) => {
                                        error!("Failed to deserialize {e}");
                                    }
                                }
                            },
                            Err(e) => println!("{e}"),
                        }
                    },
                    control = control_sub.recv_async() => {
                        match control {
                            Ok(control) => {
                                match from_bytes::<ControlRequest>(&control.value.payload.contiguous()) {
                                    Ok(control) => {
                                        let _ = control_link.0.send(control);
                                    },
                                    Err(e) => {
                                        error!("Failed to deserialize {e}");
                                    }
                                }
                            },
                            Err(e) => println!("{e}"),
                        }
                    },
                    Some(msg) = control_link.1.recv() => {
                        let data = postcard::to_allocvec(&msg).unwrap();
                        if let Err(e) = control_pub.send(data).await {
                            error!("{e}")
                        }
                    }
                    Ok(_) = pos.changed() => {
                        let update = GlobalPosition::from_vehicle(pos.borrow().to_owned());
                        let data = postcard::to_allocvec(&update).unwrap();
                        if let Err(e) = pos_pub.send(data).await {
                            error!("{e}")
                        }
                    }
                    Ok(_) = step_out.changed() => {
                        let step = *step_out.borrow();
                        let data = postcard::to_allocvec(&step).unwrap();
                        if let Err(e) = step_pub.send(data).await {
                            error!("{e}")
                        }

                    }
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            #[allow(unreachable_code)]
            Ok::<_, ZenohError>(())
        });
        ArgusLink
    }
}
