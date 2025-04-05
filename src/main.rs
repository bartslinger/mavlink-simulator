use mavlink::MavHeader;
use std::sync::Arc;

mod fixed_wing;
mod simulator;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    tracing::info!("Hello, world!");

    let simulator = simulator::Simulator::new();

    let mut header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 42,
    };
    let conn = Arc::new(
        mavlink::connect_async::<mavlink::ardupilotmega::MavMessage>("udpout:127.0.0.1:14550")
            .await?,
    );
    let uplink = conn.clone();
    let downlink = conn;

    let (uplink_tx, uplink_rx) = tokio::sync::mpsc::channel(10);
    let (downlink_tx, mut downlink_rx) =
        tokio::sync::mpsc::channel::<mavlink::ardupilotmega::MavMessage>(10);

    // Downlink task receives from simulator and sends to UDP
    let downlink_task = async move {
        loop {
            if let Some(message) = downlink_rx.recv().await {
                header.sequence = header.sequence.wrapping_add(1);
                if let Err(e) = downlink.send(&header, &message).await {
                    tracing::error!("Failed to send downlink message: {:?}", e);
                }
            } else {
                tracing::info!("Downlink channel closed");
                break;
            }
        }
    };

    // Uplink task receives on UDP and forwards to simulator
    let uplink_task = async move {
        loop {
            if let Ok((_header, message)) = uplink.recv().await {
                if let Err(e) = uplink_tx.try_send(message) {
                    tracing::error!("Failed to forward uplink message: {:?}", e);
                }
            } else {
                tracing::info!("Uplink channel closed");
                break;
            }
        }
    };

    tokio::select! {
        v = simulator.run(downlink_tx, uplink_rx) => {
            anyhow::bail!("Simulator stopped: {:?}", v);
        }
        v = downlink_task => {
            anyhow::bail!("Downlink task stopped: {:?}", v);
        }
        v = uplink_task => {
            anyhow::bail!("Uplink task stopped: {:?}", v);
        }
    }
}
