#![allow(non_snake_case)]
use super::rigid_body::{DynamicsModel, Forces, Moments, State};
use std::f64::consts::PI;

const INPUTS: usize = 5;
const ADDITIONAL_OUTPUTS: usize = 2;

// Constants
const RHO: f64 = 1.225; // Air density (kg/m^3)
const G: f64 = 9.80665; // Gravitational acceleration (m/s^2)

// Airplane geometry
const CBAR: f64 = 0.154; // Mean Aerodynamic Chord (m)

// const B: f64 = 0.980; // Wing span (m)

const S: f64 = 0.147; // Wing planform area (m^2)

const AR: f64 = 6.524; // Aspect ratio
const E: f64 = 0.9; // Oswald efficiency factor

const U_MAX: f64 = 0.625 * G; // Maximum thrust provided by one engine (N)

const CL0: f64 = 0.22816; // Lift coefficient at zero angle of attack
const CLA: f64 = 4.93732; // Lift curve slope (rad^-1)

const CM0: f64 = -0.00675; // Moment coefficient at zero angle of attack
const CMA: f64 = -0.78035; // Moment curve slope (rad^-1)
const CMQ: f64 = -10.78603; // Moment coefficient at pitch rate (rad^-1)

const CD0: f64 = 0.025; // Zero-lift drag coefficient

const CMDE: f64 = -1.17862; // Moment coefficient at elevator deflection (rad^-1)

pub struct ZohdAltusModel {}

impl DynamicsModel<INPUTS, ADDITIONAL_OUTPUTS> for ZohdAltusModel {
    // https://github.com/clum/YouTube/blob/85e5e4e2c4815ee8e4893faabf2935f936ee2649/Controls28/RCAM_model.m
    fn mass(&self) -> f64 {
        0.980
    }

    fn inertia(&self) -> nalgebra::Matrix3<f64> {
        nalgebra::Matrix3::new(0.0161, 0.0, 0.0, 0.0, 0.023, 0.0, 0.0, 0.0, 0.035)
    }

    fn input_names(&self) -> [&'static str; INPUTS] {
        ["d_a", "d_e", "d_r", "d_th1", "d_th2"]
    }

    fn output_names(&self) -> [&'static str; ADDITIONAL_OUTPUTS] {
        ["Va", "alpha"]
    }

    fn compute_forces_and_moments(
        &self,
        state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        control_input: &nalgebra::SVector<f64, INPUTS>,
    ) -> (Forces, Moments, nalgebra::SVector<f64, ADDITIONAL_OUTPUTS>) {
        let (u, v, w) = (state[0], state[1], state[2]);
        let (_p, q, _r) = (state[3], state[4], state[5]);

        // Define vectors
        let V_b = nalgebra::Vector3::new(u, v, w);

        // let d_a = control_input[0]; // d_A (aileron)
        let d_e = control_input[1]; // d_e (elevator)
                                    // let d_r = control_input[2]; // d_R (rudder)
        let d_th1 = control_input[3]; // d_th1 (throttle 1)
        let d_th2 = control_input[4]; // d_th2 (throttle 2)

        // ---------------INTERMEDIATE VARIABLES------------------------
        // Calculate airspeed
        let V_a = V_b.norm();

        // Calculate alpha and beta
        let alpha = w.atan2(u);
        let _beta = (v / V_a).asin();

        // Calculate dynamic pressure
        let dynamic_pressure = 0.5 * RHO * V_a.powi(2);

        // Calculate lift in the stability frame (sf)
        let CL = CL0 + CLA * alpha;
        let L_sf = CL * dynamic_pressure * S;

        // Ignoring lift by the tail

        // Induced drag
        let CDi = CL.powi(2) / (PI * AR * E);

        let D_sf = (CD0 + CDi) * dynamic_pressure * S;

        // Lift in body frame (bf)
        let L_bf = nalgebra::Vector3::new(L_sf * alpha.sin(), 0.0, -L_sf * alpha.cos());

        // Drag in body frame (bf)
        let D_bf = nalgebra::Vector3::new(-D_sf * alpha.cos(), 0.0, -D_sf * alpha.sin());

        // Thrust in body frame (bf)
        let T_bf = nalgebra::Vector3::new((d_th1 + d_th2) * U_MAX, 0.0, 0.0);

        // Moments

        let q_hat = q * CBAR / (2.0 * V_a);
        let Cm = CM0 + CMA * alpha + CMQ * q_hat + CMDE * d_e;
        let M_sf = Cm * dynamic_pressure * S * CBAR;
        let M_bf = M_sf;

        let g_ned = nalgebra::Vector3::new(0.0, 0.0, G);
        let g_bf = rotation_matrix.transpose() * g_ned;
        let Fg_bf = self.mass() * g_bf;

        let F_bf = Fg_bf + L_bf + D_bf + T_bf;
        let M_bf = nalgebra::Vector3::new(0.0, M_bf, 0.0);

        (
            // nalgebra::Vector3::zeros(),
            // nalgebra::Vector3::zeros(),
            F_bf,
            M_bf,
            nalgebra::SVector::<f64, ADDITIONAL_OUTPUTS>::new(V_a, alpha),
        )
    }
}
