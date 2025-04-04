use crate::coordinate_systems::{AngleExt, LLA};
use crate::fixed_wing::FixedWing;

pub struct Simulator {}

impl Simulator {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn run(
        &self,
        downlink_tx: tokio::sync::mpsc::Sender<u32>,
        mut uplink_rx: tokio::sync::mpsc::Receiver<u32>,
    ) -> Result<(), anyhow::Error> {
        let mut fixed_wing = FixedWing::new(LLA::new(52.0, 4.5, 100.0), 25.0.degrees());
        let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(100));
        loop {
            // tokio::select! code can't be auto-formatted, so using this enum workaround instead
            enum Trigger {
                Interval,
                Uplink(Option<u32>),
            }
            let trigger = tokio::select! {
                _ = interval.tick() => Trigger::Interval,
                v = uplink_rx.recv() => Trigger::Uplink(v),
            };
            match trigger {
                Trigger::Interval => {
                    fixed_wing.simulate(0.1);
                    tracing::info!("Local Position: {:?}", fixed_wing.local_position());
                }
                Trigger::Uplink(v) => {
                    if let Some(v) = v {
                        tracing::info!("Uplink: {:?}", v);
                        let send_result = downlink_tx.try_send(v);
                    } else {
                        tracing::info!("Uplink channel closed");
                        anyhow::bail!("Uplink channel closed");
                    }
                }
            }
        }
    }
}
