use std::{convert::identity, error::Error, marker::PhantomData, sync::Arc, time::Duration};

use anyhow::{anyhow, Context};
use nalgebra::Vector3;
use postcard::from_bytes;
use tokio::{
    select,
    sync::{mpsc, watch},
    time::sleep,
};
use tracing::{error, info};
use zenoh::{
    config::{self, EndPoint},
    pubsub::Publisher,
    Session,
};

use argus_common::{
    interface::{
        IControlRequest, IControlResponse, IGlobalPosition, ILocalPosition, IMissionStep,
        IMissionUpdate, IVelocity, IYaw, Interface,
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
            mpsc::Receiver<ControlResponse>,
        ),
        machine: String,
    ) -> Self {
        tokio::spawn(async move {
            let m = machine;
            let mut config = config::Config::default();

            let interface = "tailscale0";

            config
                .listen
                .endpoints
                .set(vec![EndPoint::new("tcp", "0.0.0.0:0", "", "")?])
                .emap()?;

            config.transport.unicast.set_max_links(1).emap()?;

            let session = Arc::new(zenoh::open(config).await?);

            let zid = session.zid();

            info!(
                "GCS Link: Enabled on interface {interface}, Access Control: Yes, Identity {zid}"
            );

            let global_pos_pub: Pub<IGlobalPosition> = session.publisher(&m).await?;
            let local_pos_pub: Pub<ILocalPosition> = session.publisher(&m).await?;
            let yaw_pub: Pub<IYaw> = session.publisher(&m).await?;
            let velocity_pub: Pub<IVelocity> = session.publisher(&m).await?;
            let step_pub: Pub<IMissionStep> = session.publisher(&m).await?;
            let control_pub: Pub<IControlResponse> = session.publisher(&m).await?;

            let mut mission_sub: Sub<IMissionUpdate> = session.subscriber(&m).await?;
            let mut control_sub: Sub<IControlRequest> = session.subscriber(&m).await?;

            let global_pos = subs.global_position.subscribe();
            let local_pos = subs.local_position.subscribe();
            let attitude = subs.vehicle_attitude.subscribe();

            Self::bridge(local_pos, local_pos_pub, |p| LocalPosition::from_vehicle(p));
            Self::bridge(global_pos, global_pos_pub, |p| {
                GlobalPosition::from_vehicle(p)
            });

            Self::bridge(attitude, yaw_pub, |a| attitude_to_yaw(&a));

            Self::bridge(step_out, step_pub, identity);

            let local_pos = subs.local_position.subscribe();

            Self::bridge(local_pos, velocity_pub, |p| {
                Vector3::new(p.vx.into(), p.vy.into(), p.vz.into())
            });

            loop {
                select! {
                    mission = mission_sub.pull() => {
                        match mission {
                            Ok(mission) => {
                                mission_in.send(mission).await.unwrap();
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
                        if let Err(e) = control_pub.publisher.put(data).await {
                            error!("{e}")
                        }
                    }
                }
                tokio::time::sleep(Duration::from_millis(15)).await;
            }
            #[expect(unreachable_code)]
            Ok::<_, ZenohError>(())
        });
        ArgusLink
    }

    fn bridge<I, U, F>(mut rcv: watch::Receiver<U>, publisher: Pub<I>, mut map: F)
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
                    if let Err(e) = publisher.publisher.put(data).await {
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

pub struct Sub<I: Interface> {
    tunnel: mpsc::Receiver<I::Message>,
    _phantom: PhantomData<I>,
}

impl<I: Interface> Sub<I> {
    pub async fn pull(&mut self) -> anyhow::Result<I::Message> {
        let msg = self.tunnel.recv().await.context("no more messages")?;
        Ok(msg)
    }
}

trait MakeInterface {
    async fn publisher<I: Interface>(&self, m: &str) -> anyhow::Result<Pub<I>>;
    async fn subscriber<I>(&self, m: &str) -> anyhow::Result<Sub<I>>
    where
        I: Interface,
        I::Message: Send + Sync + 'static;
}

impl MakeInterface for Arc<Session> {
    async fn publisher<I: Interface>(&self, m: &str) -> anyhow::Result<Pub<I>> {
        let res = self
            .declare_publisher(format!("{m}/{}", I::topic()))
            .await
            .map_err(|e| anyhow!("{e:?}"))?;
        Ok(Pub {
            publisher: res,
            _phantom: PhantomData,
        })
    }
    async fn subscriber<I>(&self, m: &str) -> anyhow::Result<Sub<I>>
    where
        I: Interface,
        I::Message: Send + Sync + 'static,
    {
        let (tunnel_tx, tunnel_rx) = mpsc::channel(16);

        self.declare_subscriber(format!("{m}/{}", I::topic()))
            .callback(move |sample| {
                let can_fail = || {
                    let payload = sample.payload().to_bytes();
                    let decoded =
                        from_bytes::<I::Message>(&payload).context("failed to deserialize")?;
                    tunnel_tx.try_send(decoded)?;
                    anyhow::Ok(())
                };
                if let Err(e) = can_fail() {
                    error!("{e}");
                }
            })
            .background()
            .await
            .map_err(|e| anyhow!("{e:?}"))?;

        Ok(Sub {
            tunnel: tunnel_rx,
            _phantom: PhantomData,
        })
    }
}
