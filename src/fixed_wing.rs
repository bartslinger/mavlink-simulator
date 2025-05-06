use crate::flight_dynamics::{DynamicsModel, RigidBody, State};

pub struct FixedWing<
    M: DynamicsModel<INPUTS, ADDITIONAL_OUTPUTS>,
    const INPUTS: usize,
    const ADDITIONAL_OUTPUTS: usize,
> {
    // pub model: Box<dyn DynamicsModel<INPUTS, ADDITIONAL_OUTPUTS> + Send + 'model>,
    pub body: RigidBody<M, INPUTS, ADDITIONAL_OUTPUTS>,
    pub state: State,
    pub origin: nalgebra::Vector3<f64>,
    pub local_position: nalgebra::Vector3<f64>,
    // pub local_velocity: nalgebra::Vector3<f64>,
    pub attitude: nalgebra::Vector3<f64>,
}

impl<
        M: DynamicsModel<INPUTS, ADDITIONAL_OUTPUTS>,
        const INPUTS: usize,
        const ADDITIONAL_OUTPUTS: usize,
    > FixedWing<M, INPUTS, ADDITIONAL_OUTPUTS>
{
    pub fn new(
        body: RigidBody<M, INPUTS, ADDITIONAL_OUTPUTS>,
        origin: nalgebra::Vector3<f64>,
        heading_deg: f64,
    ) -> Self {
        // moving forward with 12 m/s
        let velocity = nalgebra::Vector3::new(12.0, 0.0, 0.0);
        let heading_rad = heading_deg.to_radians();
        let rotation =
            nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::z_axis(), heading_rad);
        let local_velocity: nalgebra::Vector3<f64> = rotation * velocity;

        let state = nalgebra::SVector::<f64, 13>::zeros();

        tracing::info!("{:?}", local_velocity);

        Self {
            body,
            state,
            origin,
            local_position: nalgebra::Vector3::new(0.0, 0.0, 0.0),
            // local_velocity,
            attitude: nalgebra::Vector3::new(0.0, 0.0, heading_rad),
        }
    }

    pub fn simulate(&mut self, dt: f64) {
        let control_input = nalgebra::SVector::<f64, INPUTS>::zeros();
        let (new_state, forces, moments, outputs) = self.body.step(&self.state, &control_input, dt);

        // Update local_velocity to be in the direction of attitude
        let rotation = nalgebra::Rotation3::from_euler_angles(
            self.attitude.x,
            self.attitude.y,
            self.attitude.z,
        );
        let velocity = nalgebra::Vector3::new(12.0, 0.0, 0.0);
        let local_velocity: nalgebra::Vector3<f64> = rotation * velocity;

        // Update local_position based on local_velocity
        self.local_position += local_velocity * dt;
        tracing::info!("{:?}", self.local_position);
    }
}

pub fn global_position(
    local_position: &nalgebra::Vector3<f64>,
    origin: &nalgebra::Vector3<f64>,
) -> nalgebra::Vector3<f64> {
    let lat0_rad = origin.x.to_radians();
    let lon0_rad = origin.y.to_radians();

    // Radius of curvature in the prime vertical
    let r_n = 6378137.0;

    let d_lat = local_position.x / r_n;
    let d_lon = local_position.y / (r_n * lat0_rad.cos());

    let lat = lat0_rad + d_lat;
    let lon = lon0_rad + d_lon;
    let alt = origin.z - local_position.z;

    nalgebra::Vector3::new(lat.to_degrees(), lon.to_degrees(), alt)
}
