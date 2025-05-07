use crate::flight_dynamics::{DynamicsModel, RigidBody, State};

pub struct FixedWing<
    M: DynamicsModel<INPUTS, ADDITIONAL_OUTPUTS>,
    const INPUTS: usize,
    const ADDITIONAL_OUTPUTS: usize,
> {
    pub body: RigidBody<M, INPUTS, ADDITIONAL_OUTPUTS>,
    pub state: State,
    pub origin: nalgebra::Vector3<f64>,
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
        speed: f64,
        pitch_deg: f64,
        heading_deg: f64,
    ) -> Self {
        let pitch = pitch_deg.to_radians();
        // calculate quaternion from initial roll, pitch, yaw rotation
        let half_roll: f64 = 0.0 * 0.5;
        let half_pitch: f64 = pitch * 0.5;
        let half_yaw: f64 = heading_deg.to_radians() * 0.5;

        let cos_roll = half_roll.cos();
        let sin_roll = half_roll.sin();
        let cos_pitch = half_pitch.cos();
        let sin_pitch = half_pitch.sin();
        let cos_yaw = half_yaw.cos();
        let sin_yaw = half_yaw.sin();

        // Calculate each quaternion component
        let q0 = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
        let q1 = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
        let q2 = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
        let q3 = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;

        // Split velocity components u,w
        // flying straight and level
        let u = speed * (pitch).cos();
        let w = speed * (pitch).sin();

        let state = nalgebra::SVector::<f64, 13>::from([
            u, 0.0, w, 0.0, 0.0, 0.0, q0, q1, q2, q3, 0.0, 0.0, 0.0,
        ]);

        Self {
            body,
            state,
            origin,
        }
    }

    pub fn simulate(&mut self, dt: f64) {
        let mut control_input = nalgebra::SVector::<f64, INPUTS>::zeros();
        control_input[3] = 0.15;
        control_input[4] = 0.15;
        let (new_state, forces, moments, outputs) = self.body.step(&self.state, &control_input, dt);
        self.state = new_state;
    }

    pub fn local_position(&self) -> nalgebra::Vector3<f64> {
        nalgebra::Vector3::new(self.state[10], self.state[11], self.state[12])
    }

    pub fn rpy_deg(&self) -> nalgebra::Vector3<f64> {
        let q =
            nalgebra::Quaternion::new(self.state[6], self.state[7], self.state[8], self.state[9]);
        let (roll, pitch, yaw) = nalgebra::UnitQuaternion::from_quaternion(q).euler_angles();
        let roll_deg = roll.to_degrees();
        let pitch_deg = pitch.to_degrees();
        // yaw between 0 and 360
        let yaw_deg = yaw.to_degrees();
        let yaw_deg = if yaw_deg < 0.0 {
            360.0 + yaw_deg
        } else {
            yaw_deg
        };

        nalgebra::Vector3::new(roll_deg, pitch_deg, yaw_deg)
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
