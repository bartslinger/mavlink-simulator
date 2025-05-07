// #[derive(Copy, Clone, Debug)]
// pub struct State {
//     u: f64,
//     v: f64,
//     w: f64,
//     p: f64,
//     q: f64,
//     r: f64,
//     q0: f64,
//     q1: f64,
//     q2: f64,
//     q3: f64,
//     pn: f64,
//     pe: f64,
//     pd: f64,
// }

use std::fmt::Display;

pub type State = nalgebra::SVector<f64, 13>;

pub type StateDerivative = nalgebra::SVector<f64, 13>;

#[derive(Debug)]
pub struct InitialCondition {
    pub velocity: nalgebra::Vector3<f64>,
    pub angular_velocity: nalgebra::Vector3<f64>,
    pub rotation: nalgebra::Vector3<f64>,
    pub position: nalgebra::Vector3<f64>,
}

impl Display for InitialCondition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "velocity:\t{}\nangular_velocity:\t{}\nrotation:\t{}\nposition:\t{}",
            self.velocity, self.angular_velocity, self.rotation, self.position
        )
    }
}

pub type Forces = nalgebra::Vector3<f64>;
pub type Moments = nalgebra::Vector3<f64>;

pub trait DynamicsModel<const INPUTS: usize, const ADDITIONAL_OUTPUTS: usize> {
    fn mass(&self) -> f64;
    fn inertia(&self) -> nalgebra::Matrix3<f64>;

    fn input_names(&self) -> [&'static str; INPUTS];
    fn output_names(&self) -> [&'static str; ADDITIONAL_OUTPUTS];
    fn compute_forces_and_moments(
        &self,
        state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        control_input: &nalgebra::SVector<f64, INPUTS>,
    ) -> (Forces, Moments, nalgebra::SVector<f64, ADDITIONAL_OUTPUTS>);
}

pub struct RigidBody<M, const I: usize, const O: usize>
where
    M: DynamicsModel<I, O>,
{
    mass: f64,
    inertia: nalgebra::Matrix3<f64>,
    inertia_inverse: nalgebra::Matrix3<f64>,
    dynamics_model: M,
}

impl<M, const I: usize, const O: usize> RigidBody<M, I, O>
where
    M: DynamicsModel<I, O>,
{
    pub fn new(dynamics_model: M) -> Self {
        let mass = dynamics_model.mass();
        let inertia = dynamics_model.inertia();
        let inertia_inverse = inertia.try_inverse().unwrap();
        tracing::debug!("{:?}", dynamics_model.input_names());
        tracing::debug!("{:?}", dynamics_model.output_names());

        Self {
            mass,
            inertia,
            inertia_inverse,
            dynamics_model,
        }
    }

    fn compute_state_derivative(
        &self,
        state: &State,
        control_input: &nalgebra::SVector<f64, I>,
    ) -> (StateDerivative, Forces, Moments, nalgebra::SVector<f64, O>) {
        let (u, v, w) = (state[0], state[1], state[2]);
        let (p, q, r) = (state[3], state[4], state[5]);
        let (q0, q1, q2, q3) = (state[6], state[7], state[8], state[9]);
        // let (pn, pe, pd) = (state[10], state[11], state[12]);

        let velocity_body = nalgebra::Vector3::new(u, v, w);
        let omega = nalgebra::Vector3::new(p, q, r);

        let rotation_matrix = rotation_matrix_from_quaternion(q0, q1, q2, q3);
        // let inverse_rotation_matrix = rotation_matrix.transpose();

        let velocity_ned = rotation_matrix * velocity_body;

        let (forces, moments, additional_outputs) =
            self.dynamics_model
                .compute_forces_and_moments(state, &rotation_matrix, control_input);

        let part1 = self.inertia * omega;
        let coriolis = omega.cross(&part1);
        let omega_dot = self.inertia_inverse * (moments - coriolis);

        let u_dot = forces[0] / self.mass - w * q + v * r;
        let v_dot = forces[1] / self.mass - u * r + w * p;
        let w_dot = forces[2] / self.mass - v * p + u * q;
        let p_dot = omega_dot[0];
        let q_dot = omega_dot[1];
        let r_dot = omega_dot[2];
        let q0_dot = -0.5 * (q1 * p + q2 * q + q3 * r);
        let q1_dot = 0.5 * (q0 * p + q2 * r - q3 * q);
        let q2_dot = 0.5 * (q0 * q + q3 * p - q1 * r);
        let q3_dot = 0.5 * (q0 * r + q1 * q - q2 * p);
        let pn_dot = velocity_ned[0];
        let pe_dot = velocity_ned[1];
        let pd_dot = velocity_ned[2];

        let state_derivative: StateDerivative = nalgebra::SVector::<f64, 13>::from([
            u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, q0_dot, q1_dot, q2_dot, q3_dot, pn_dot,
            pe_dot, pd_dot,
        ]);
        (state_derivative, forces, moments, additional_outputs)
    }

    fn runge_kutta_propagation(
        &self,
        state: &State,
        control_input: &nalgebra::SVector<f64, I>,
        dt: f64,
    ) -> (State, Forces, Moments, nalgebra::SVector<f64, O>) {
        let (k1, f1, m1, a1) = self.compute_state_derivative(state, control_input);
        let (k2, f2, m2, a2) =
            self.compute_state_derivative(&(state + k1 * dt / 2.0), control_input);
        let (k3, f3, m3, a3) =
            self.compute_state_derivative(&(state + k2 * dt / 2.0), control_input);
        let (k4, f4, m4, a4) = self.compute_state_derivative(&(state + k3 * dt), control_input);
        let next_state = state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt / 6.0;
        let forces = (f1 + 2.0 * f2 + 2.0 * f3 + f4) / 6.0;
        let moments = (m1 + 2.0 * m2 + 2.0 * m3 + m4) / 6.0;
        let additional_outputs = (a1 + 2.0 * a2 + 2.0 * a3 + a4) / 6.0;
        (next_state, forces, moments, additional_outputs)
    }

    pub fn step(
        &self,
        state: &State,
        control_input: &nalgebra::SVector<f64, I>,
        dt: f64,
    ) -> (State, Forces, Moments, nalgebra::SVector<f64, O>) {
        let (state, forces, moments, additional_outputs) =
            self.runge_kutta_propagation(state, control_input, dt);
        let state = normalize_quaternion(state);
        (state, forces, moments, additional_outputs)
    }
}

fn normalize_quaternion(mut state: State) -> State {
    let q = nalgebra::Vector4::new(state[6], state[7], state[8], state[9]);
    let q_normalized = q.normalize();
    state[6] = q_normalized[0];
    state[7] = q_normalized[1];
    state[8] = q_normalized[2];
    state[9] = q_normalized[3];
    state
}

pub fn rotation_matrix_from_quaternion(
    q0: f64,
    q1: f64,
    q2: f64,
    q3: f64,
) -> nalgebra::Matrix3<f64> {
    let r11 = q0.powi(2) + q1.powi(2) - q2.powi(2) - q3.powi(2);
    let r22 = q0.powi(2) - q1.powi(2) + q2.powi(2) - q3.powi(2);
    let r33 = q0.powi(2) - q1.powi(2) - q2.powi(2) + q3.powi(2);
    let r12 = 2.0 * (q1 * q2 - q0 * q3);
    let r21 = 2.0 * (q1 * q2 + q0 * q3);
    let r13 = 2.0 * (q1 * q3 + q0 * q2);
    let r31 = 2.0 * (q1 * q3 - q0 * q2);
    let r23 = 2.0 * (q2 * q3 - q0 * q1);
    let r32 = 2.0 * (q2 * q3 + q0 * q1);
    nalgebra::Matrix3::<f64>::new(r11, r12, r13, r21, r22, r23, r31, r32, r33)
}
