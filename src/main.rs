use crate::coordinate_systems::{AngleExt, LLA};
use crate::fixed_wing::FixedWing;
use mavlink::common::{MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, HEARTBEAT_DATA};
use mavlink::{MAVLinkV2MessageRaw, MavHeader};

mod coordinate_systems;
mod fixed_wing;
mod simulator;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    tracing::info!("Hello, world!");

    let simulator = simulator::Simulator::new();

    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    };
    let heartbeat_data = HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_FIXED_WING,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    };
    let mut message = MAVLinkV2MessageRaw::new();
    message.serialize_message_data(header, &heartbeat_data);
    let bytes = message.raw_bytes();
    tracing::info!("Bytes: {:?}", bytes);
    let mavlink_socket = tokio::net::UdpSocket::bind("0.0.0.0:0").await?;

    loop {
        let x = mavlink_socket.send_to(bytes, "127.0.0.1:14550").await?;
        tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        tracing::info!("Sent {} bytes", x);
    }

    let (uplink_tx, uplink_rx) = tokio::sync::mpsc::channel(10);
    let (downlink_tx, mut downlink_rx) = tokio::sync::mpsc::channel(10);

    let interface_task = async move {
        let mut counter = 0;
        loop {
            // send something to the sim
            let send_result = uplink_tx.send(counter).await;
            if let Err(_) = send_result {
                tracing::info!("Uplink channel closed");
                break;
            }
            // receive something
            let recv_result = downlink_rx.recv().await;
            if let Some(v) = recv_result {
                tracing::info!("Downlink: {:?}", v);
            } else {
                tracing::info!("Downlink channel closed");
                break;
            }
            counter += 1;
            tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        }
    };

    tokio::select! {
        v = simulator.run(downlink_tx, uplink_rx) => {
            tracing::warn!("Simulator stopped: {:?}", v);
        }
        v = interface_task => {
            tracing::warn!("Interface task stopped: {:?}", v);
        }
    }
    Ok(())
}
