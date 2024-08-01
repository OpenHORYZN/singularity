use std::{
    convert::identity,
    error::Error,
    sync::{mpsc, Arc},
    time::Duration,
};

use anyhow::anyhow;
use futures_util::SinkExt;
use nalgebra::{Quaternion, UnitQuaternion};
use postcard::from_bytes;
use serde::Serialize;
use tokio::{select, sync::watch, time::sleep};
use tracing::error;
use zenoh::{
    config::{AclConfigRules, Action, InterceptorFlow, Permission},
    prelude::r#async::*,
    publication::Publisher,
};

use argus_common::{ControlRequest, ControlResponse, GlobalPosition, LocalPosition, MissionNode};

use crate::{
    topics::Subscribers,
    util::{GlobalPositionFeatures, LocalPositionFeatures},
};

pub type ZenohError = Box<dyn Error + Send + Sync>;

pub struct ArgusLink;

impl ArgusLink {
    pub async fn init(
        subs: Arc<Subscribers>,
        mission_in: mpsc::Sender<Vec<MissionNode>>,
        step_out: watch::Receiver<i32>,
        mut control_link: (
            mpsc::Sender<ControlRequest>,
            tokio::sync::mpsc::Receiver<ControlResponse>,
        ),
        machine: String,
    ) -> Self {
        tokio::spawn(async move {
            let t = |o: &str| format!("{machine}/{o}");
            let mut config = config::default();

            Self::set_acl(&mut config)?;

            let session = Arc::new(zenoh::open(config).res().await?);
            let global_pos_pub = session
                .declare_publisher(t("global_position"))
                .res()
                .await?;
            let local_pos_pub = session.declare_publisher(t("local_position")).res().await?;
            let yaw_pub = session.declare_publisher(t("yaw")).res().await?;
            let step_pub = session.declare_publisher(t("mission/step")).res().await?;
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

            let global_pos = subs.global_position.subscribe();
            let local_pos = subs.local_position.subscribe();
            let attitude = subs.vehicle_attitude.subscribe();

            Self::bridge(local_pos, local_pos_pub, |p| LocalPosition::from_vehicle(p));
            Self::bridge(global_pos, global_pos_pub, |p| {
                GlobalPosition::from_vehicle(p)
            });

            Self::bridge(attitude, yaw_pub, |a| {
                let [w, x, y, z] = a.q;
                let quat = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
                let (_, _, yaw) = quat.euler_angles();
                yaw
            });

            Self::bridge(step_out, step_pub, identity);

            loop {
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
                }
                tokio::time::sleep(Duration::from_millis(15)).await;
            }
            #[allow(unreachable_code)]
            Ok::<_, ZenohError>(())
        });
        ArgusLink
    }

    fn bridge<T, U, F>(mut rcv: watch::Receiver<T>, mut publisher: Publisher<'static>, mut map: F)
    where
        F: FnMut(T) -> U + Send + 'static,
        T: ToOwned<Owned = T> + Send + Sync + 'static,
        U: Serialize + Send + Sync + 'static,
    {
        tokio::spawn(async move {
            loop {
                if let Ok(_) = rcv.changed().await {
                    let update = map(rcv.borrow().to_owned());
                    let data = postcard::to_allocvec(&update).unwrap();
                    if let Err(e) = publisher.send(data).await {
                        error!("{e}")
                    }
                }
                sleep(Duration::from_millis(15)).await;
            }
        });
    }

    fn set_acl(config: &mut Config) -> Result<(), anyhow::Error> {
        let all_actions = [
            Action::Put,
            Action::DeclareSubscriber,
            Action::Get,
            Action::DeclareQueryable,
        ];
        config
            .access_control
            .set_default_permission(Permission::Deny)
            .map_err(|e| anyhow!("{e:?}"))?;
        config
            .access_control
            .set_rules(Some(vec![AclConfigRules {
                interfaces: Some(vec!["tailscale0".into(), "lo".into()]),
                actions: all_actions.to_vec(),
                key_exprs: vec!["**".into()],
                flows: Some(vec![InterceptorFlow::Ingress, InterceptorFlow::Egress]),
                permission: Permission::Allow,
            }]))
            .map_err(|e| anyhow!("{e:?}"))?;
        config
            .access_control
            .set_enabled(true)
            .map_err(|e| anyhow!("{e:?}"))?;
        Ok(())
    }
}
