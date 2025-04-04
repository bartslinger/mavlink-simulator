use crate::coordinate_systems::{AngleExt, LLA};
use crate::fixed_wing::FixedWing;

mod coordinate_systems;
mod fixed_wing;
mod realtime_sim;

fn main() {
    // Initialize logging
    tracing_subscriber::fmt::init();

    tracing::info!("Hello, world!");

    let mut fixed_wing = FixedWing::new(LLA::new(52.0, 4.5, 100.0), 25.0.degrees());

    for i in 0..10 {
        fixed_wing.simulate(0.1);
        tracing::info!("Local Position: {:?}", fixed_wing.local_position());
    }
}
