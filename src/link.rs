use std::{
    convert::identity,
    error::Error,
    marker::PhantomData,
    sync::{mpsc, Arc},
    time::Duration,
};

use anyhow::{anyhow, bail};
use futures_util::SinkExt;
use postcard::from_bytes;
use tokio::{select, sync::watch, time::sleep};
use tracing::{error, info};
use zenoh::{
    config::{AclConfigRules, Action, InterceptorFlow, Permission},
    prelude::r#async::*,
    publication::Publisher,
    subscriber::FlumeSubscriber,
};

use argus_common::{
    interface::{
        IControlRequest, IControlResponse, IGlobalPosition, ILocalPosition, IMissionStep,
        IMissionUpdate, IYaw, Interface,
    },
    ControlRequest, ControlResponse, GlobalPosition, LocalPosition, MissionPlan,
};

use crate::{
    topics::Subscribers,
    util::{attitude_to_yaw, GlobalPositionFeatures, LocalPositionFeatures, MapErr},
};

pub type ZenohError = Box<dyn Error + Send + Sync>;

pub struct ArgusLink;

impl ArgusLink {
    pub async fn init(
        subs: Arc<Subscribers>,
        mission_in: mpsc::Sender<MissionPlan>,
        step_out: watch::Receiver<i32>,
        mut control_link: (
            mpsc::Sender<ControlRequest>,
            tokio::sync::mpsc::Receiver<ControlResponse>,
        ),
        machine: String,
    ) -> Self {
        tokio::spawn(async move {
            let m = machine;
            let mut config = config::default();

            Self::set_acl(&mut config)?;

            let interface = "tailscale0";

            config.listen.endpoints = vec![
                EndPoint::new("udp", "0.0.0.0:0", "", format!("iface={interface}"))?,
                EndPoint::new("tcp", "0.0.0.0:0", "", format!("iface={interface}"))?,
            ];

            config.transport.unicast.set_max_links(10).emap()?;

            let session = Arc::new(zenoh::open(config).res().await?);

            let zid = session.zid();

            info!(
                "GCS Link: Enabled on interface {interface}, Access Control: Yes, Identity {zid}"
            );

            let global_pos_pub: Pub<IGlobalPosition> = session.publisher(&m).await?;
            let local_pos_pub: Pub<ILocalPosition> = session.publisher(&m).await?;
            let yaw_pub: Pub<IYaw> = session.publisher(&m).await?;
            let step_pub: Pub<IMissionStep> = session.publisher(&m).await?;
            let mut control_pub: Pub<IControlResponse> = session.publisher(&m).await?;

            let mission_sub: Sub<IMissionUpdate> = session.subscriber(&m).await?;
            let control_sub: Sub<IControlRequest> = session.subscriber(&m).await?;

            let global_pos = subs.global_position.subscribe();
            let local_pos = subs.local_position.subscribe();
            let attitude = subs.vehicle_attitude.subscribe();

            Self::bridge(local_pos, local_pos_pub, |p| LocalPosition::from_vehicle(p));
            Self::bridge(global_pos, global_pos_pub, |p| {
                GlobalPosition::from_vehicle(p)
            });

            Self::bridge(attitude, yaw_pub, |a| attitude_to_yaw(&a));

            Self::bridge(step_out, step_pub, identity);

            loop {
                select! {
                    mission = mission_sub.pull() => {
                        match mission {
                            Ok(mission) => {
                                mission_in.send(mission).unwrap();
                            },
                            Err(e) => println!("{e}"),
                        }
                    },
                    control = control_sub.pull() => {
                        match control {
                            Ok(control) => {
                                let _ = control_link.0.send(control);
                            },
                            Err(e) => println!("{e}"),
                        }
                    },
                    Some(msg) = control_link.1.recv() => {
                        let data = postcard::to_allocvec(&msg).unwrap();
                        if let Err(e) = control_pub.publisher.send(data).await {
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
            .emap()?;
        config
            .access_control
            .set_rules(Some(vec![AclConfigRules {
                interfaces: Some(vec!["tailscale0".into(), "lo".into()]),
                actions: all_actions.to_vec(),
                key_exprs: vec!["**".into()],
                flows: Some(vec![InterceptorFlow::Ingress, InterceptorFlow::Egress]),
                permission: Permission::Allow,
            }]))
            .emap()?;
        config.access_control.set_enabled(true).emap()?;
        Ok(())
    }

    fn bridge<I, U, F>(mut rcv: watch::Receiver<U>, mut publisher: Pub<I>, mut map: F)
    where
        I: Interface,
        I::Message: Clone + Send + Sync + 'static,
        U: Clone + Send + Sync + 'static,
        F: FnMut(U) -> I::Message + Send + 'static,
    {
        tokio::spawn(async move {
            loop {
                if let Ok(_) = rcv.changed().await {
                    let update = map(rcv.borrow().to_owned());
                    let data = postcard::to_allocvec(&update).unwrap();
                    if let Err(e) = publisher.publisher.send(data).await {
                        error!("{e}")
                    }
                }
                sleep(Duration::from_millis(15)).await;
            }
        });
    }
}

pub struct Pub<I> {
    publisher: Publisher<'static>,
    _phantom: PhantomData<I>,
}

pub struct Sub<I> {
    subscriber: FlumeSubscriber<'static>,
    _phantom: PhantomData<I>,
}

impl<I: Interface> Sub<I> {
    pub async fn pull(&self) -> anyhow::Result<I::Message> {
        let msg = self.subscriber.recv_async().await;
        match msg {
            Ok(msg) => match from_bytes(&msg.value.payload.contiguous()) {
                Ok(msg_c) => return Ok(msg_c),
                Err(e) => {
                    bail!("Failed to deserialize {e}");
                }
            },
            Err(e) => bail!("{e}"),
        };
    }
}

trait MakeInterface {
    async fn publisher<I: Interface>(&self, m: &str) -> anyhow::Result<Pub<I>>;
    async fn subscriber<I: Interface>(&self, m: &str) -> anyhow::Result<Sub<I>>;
}

impl MakeInterface for Arc<Session> {
    async fn publisher<I: Interface>(&self, m: &str) -> anyhow::Result<Pub<I>> {
        let res = self
            .declare_publisher(format!("{m}/{}", I::topic()))
            .res()
            .await
            .map_err(|e| anyhow!("{e:?}"))?;
        Ok(Pub {
            publisher: res,
            _phantom: PhantomData,
        })
    }
    async fn subscriber<I: Interface>(&self, m: &str) -> anyhow::Result<Sub<I>> {
        let sub = self
            .declare_subscriber(format!("{m}/{}", I::topic()))
            .reliable()
            .res()
            .await
            .map_err(|e| anyhow!("{e:?}"))?;

        Ok(Sub {
            subscriber: sub,
            _phantom: PhantomData,
        })
    }
}
