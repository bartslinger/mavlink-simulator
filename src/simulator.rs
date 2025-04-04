use crate::coordinate_systems::{AngleExt, LLA};
use crate::fixed_wing::FixedWing;
use mavlink::common::{MavAutopilot, MavModeFlag, MavState, MavType};

pub struct Simulator {}

impl Simulator {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn run(
        &self,
        downlink_tx: tokio::sync::mpsc::Sender<mavlink::ardupilotmega::MavMessage>,
        mut uplink_rx: tokio::sync::mpsc::Receiver<mavlink::ardupilotmega::MavMessage>,
    ) -> Result<(), anyhow::Error> {
        let mut fixed_wing = FixedWing::new(LLA::new(52.0, 4.5, 100.0), 25.0.degrees());
        let mut physics_interval = tokio::time::interval(tokio::time::Duration::from_millis(100));
        let mut heartbeat_interval = tokio::time::interval(tokio::time::Duration::from_secs(2));

        loop {
            // tokio::select! code can't be auto-formatted, so using this enum workaround instead
            enum Trigger {
                PhysicsInterval,
                HeartbeatInterval,
                Uplink(Option<mavlink::ardupilotmega::MavMessage>),
            }
            let trigger = tokio::select! {
                _ = physics_interval.tick() => Trigger::PhysicsInterval,
                _ = heartbeat_interval.tick() => Trigger::HeartbeatInterval,
                v = uplink_rx.recv() => Trigger::Uplink(v),
            };
            match trigger {
                Trigger::PhysicsInterval => {
                    fixed_wing.simulate(0.1);
                }
                Trigger::HeartbeatInterval => {
                    let message = mavlink::ardupilotmega::MavMessage::HEARTBEAT(
                        mavlink::ardupilotmega::HEARTBEAT_DATA {
                            custom_mode: 0,
                            mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_FIXED_WING,
                            autopilot:
                                mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                            base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
                            system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
                            mavlink_version: 0x3,
                        },
                    );
                    let send_result = downlink_tx.try_send(message);
                    if send_result.is_err() {
                        anyhow::bail!("Downlink channel closed");
                    }
                }
                Trigger::Uplink(v) => {
                    if let Some(v) = v {
                        tracing::info!("Uplink: {:?}", v);
                    } else {
                        tracing::info!("Uplink channel closed");
                        anyhow::bail!("Uplink channel closed");
                    }
                }
            }
        }
    }
}
