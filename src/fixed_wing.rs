use crate::coordinate_systems::{IntoRadians, LLA, NED, RPY};

pub struct FixedWing {
    origin: LLA,
    local_position: nalgebra::Vector3<f64>,
    local_velocity: nalgebra::Vector3<f64>,
    attitude: RPY,
}

impl FixedWing {
    pub fn new<U: IntoRadians>(origin: LLA, heading: U) -> Self {
        // moving forward with 12 m/s
        let velocity = nalgebra::Vector3::new(12.0, 0.0, 0.0);
        let heading_rad = heading.to_radians();
        let rotation =
            nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::z_axis(), heading_rad);
        let local_velocity: nalgebra::Vector3<f64> = rotation * velocity;

        tracing::info!("{:?}", local_velocity);

        Self {
            origin,
            local_position: nalgebra::Vector3::new(0.0, 0.0, 0.0),
            local_velocity,
            attitude: RPY::new(0.0, 0.0, heading_rad),
        }
    }

    pub fn local_position(&self) -> &nalgebra::Vector3<f64> {
        &self.local_position
    }

    pub fn simulate(&mut self, dt: f64) {
        // Simulate the fixed wing dynamics here
        // For now use super simple simulation dynamics. Just move forward in the direction of
        // attitude.

        // Update local_velocity to be in the direction of attitude
        let rotation = nalgebra::Rotation3::from_euler_angles(
            self.attitude.roll(),
            self.attitude.pitch(),
            self.attitude.yaw(),
        );
        let velocity = nalgebra::Vector3::new(12.0, 0.0, 0.0);
        let local_velocity: nalgebra::Vector3<f64> = rotation * velocity;

        // Update local_position based on local_velocity
        self.local_position += local_velocity * dt;
    }
}
