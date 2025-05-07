use crate::fixed_wing::{global_position, ControlInput, FixedWing};
use crate::flight_dynamics::zohd_altus::ZohdAltusModel;
use crate::flight_dynamics::RigidBody;

pub struct Simulator {}

impl Simulator {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn run(
        &self,
        downlink_tx: tokio::sync::mpsc::Sender<mavlink::ardupilotmega::MavMessage>,
        mut uplink_rx: tokio::sync::mpsc::Receiver<mavlink::ardupilotmega::MavMessage>,
    ) -> ! {
        let body = RigidBody::new(ZohdAltusModel {});
        let mut fixed_wing = FixedWing::new(
            body,
            nalgebra::Vector3::new(53.25230577819744, 5.06370256065469, 100.0),
            18.423,
            1.0,
            45.0,
        );
        let mut physics_interval = tokio::time::interval(tokio::time::Duration::from_millis(5));
        let mut broadcast_1hz_interval = tokio::time::interval(tokio::time::Duration::from_secs(2));
        let mut broadcast_5hz_interval =
            tokio::time::interval(tokio::time::Duration::from_millis(200));

        loop {
            // tokio::select! code can't be auto-formatted, so using this enum workaround instead
            enum Trigger {
                PhysicsInterval,
                Broadcast1HzInterval,
                Broadcast5HzInterval,
                Uplink(Option<mavlink::ardupilotmega::MavMessage>),
            }
            let trigger = tokio::select! {
                _ = physics_interval.tick() => Trigger::PhysicsInterval,
                _ = broadcast_1hz_interval.tick() => Trigger::Broadcast1HzInterval,
                _ = broadcast_5hz_interval.tick() => Trigger::Broadcast5HzInterval,
                v = uplink_rx.recv() => Trigger::Uplink(v),
            };
            match trigger {
                Trigger::PhysicsInterval => {
                    fixed_wing.simulate(
                        physics_interval.period().as_secs_f64(),
                        ControlInput {
                            roll: 0.0,
                            pitch: 0.05,
                            yaw: 0.0,
                            throttle: 0.15,
                        },
                    );
                }
                Trigger::Broadcast1HzInterval => {
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
                    if let Err(e) = downlink_tx.try_send(message) {
                        tracing::error!("Downlink channel error: {:?}", e);
                    }
                }
                Trigger::Broadcast5HzInterval => {
                    let global_position =
                        global_position(&fixed_wing.local_position(), &fixed_wing.origin);
                    let rpy_deg = fixed_wing.rpy_deg();
                    tracing::info!(
                        "Pitch: {:.2}\t Speed: {:.2}",
                        rpy_deg[1],
                        fixed_wing.state[0]
                    );
                    let message = mavlink::ardupilotmega::MavMessage::GLOBAL_POSITION_INT(
                        mavlink::ardupilotmega::GLOBAL_POSITION_INT_DATA {
                            time_boot_ms: 0,
                            lat: (global_position.x * 10_000_000.0).round() as i32,
                            lon: (global_position.y * 10_000_000.0).round() as i32,
                            alt: (global_position.z * 1000.0).round() as i32,
                            relative_alt: (global_position.z * 1000.0).round() as i32,
                            vx: 120,
                            vy: 0,
                            vz: 0,
                            hdg: (rpy_deg[2] * 100.0) as u16,
                        },
                    );
                    if let Err(e) = downlink_tx.try_send(message) {
                        tracing::error!("Downlink channel error: {:?}", e);
                    }
                }
                Trigger::Uplink(v) => {
                    if let Some(v) = v {
                        tracing::info!("Uplink: {:?}", v);
                    } else {
                        tracing::info!("Uplink channel closed");
                    }
                }
            }
        }
    }
}
