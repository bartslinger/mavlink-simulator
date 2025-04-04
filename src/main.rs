use crate::coordinate_systems::{AngleExt, LLA};
use crate::fixed_wing::FixedWing;

mod coordinate_systems;
mod fixed_wing;
mod simulator;

#[tokio::main]
async fn main() {
    // Initialize logging
    tracing_subscriber::fmt::init();

    tracing::info!("Hello, world!");

    let simulator = simulator::Simulator::new();

    let (uplink_tx, uplink_rx) = tokio::sync::mpsc::channel(10);
    let (downlink_tx, downlink_rx) = tokio::sync::mpsc::channel(10);

    tokio::select! {
        v = simulator.run(downlink_tx, uplink_rx) => {
            tracing::warn!("Simulator stopped: {:?}", v);
        }
    }
}
