#![allow(non_snake_case)]
use super::rigid_body::{DynamicsModel, Forces, Moments, State};
use std::f64::consts::PI;

const INPUTS: usize = 5;
const ADDITIONAL_OUTPUTS: usize = 2;

// Constants
const CBAR: f64 = 0.154; // Mean Aerodynamic Chord (m)

const B: f64 = 0.980; // Wing span (m)

const S: f64 = 0.147; // Wing planform area (m^2)

const AR: f64 = 6.524; // Aspect ratio
const E: f64 = 0.9; // Oswald efficiency factor

// const S_TAIL: f64 = 0.21 * 0.06; // Tail planform area (m^2)
// const X_CG: f64 = -0.027; // x position of CoG wrt leading edge (m)
// const Y_CG: f64 = 0.0; // y position of CoG wrt leading edge (m)
// const Z_CG: f64 = 0.03; // z position of CoG wrt leading edge (m)
//
// // const X_AC: f64 = 0.5 * CBAR; // x position of aerodynamic center wrt leading edge (m)
// const X_AC: f64 = -0.05; // x position of aerodynamic center wrt leading edge (m)
// const Y_AC: f64 = 0.0; // y position of aerodynamic center wrt leading edge (m)
// const Z_AC: f64 = 0.0; // z position of aerodynamic center wrt leading edge (m)
//
// // Engine inputs
// const U_MAX: f64 = 0.45 * 9.80665; // maximum thrust provided by one engine (N)
//
// const X_APT1: f64 = -0.31; // x position of engine force wrt leading edge (m)
// const Y_APT1: f64 = 0.0; // y position of engine force wrt leading edge (m)
// const Z_APT1: f64 = 0.015; // z position of engine force wrt leading edge (m)
//
// // Other constants
const RHO: f64 = 1.225; // Air density (kg/m^3)
const G: f64 = 9.80665; // Gravitational acceleration (m/s^2)

const CL0: f64 = 0.22816; // Lift coefficient at zero angle of attack
const CLA: f64 = 4.93732; // Lift curve slope (rad^-1)

const CM0: f64 = -0.00675; // Moment coefficient at zero angle of attack
const CMA: f64 = -0.78035; // Moment curve slope (rad^-1)
const CMQ: f64 = -10.78603; // Moment coefficient at pitch rate (rad^-1)

const CD0: f64 = 0.025; // Zero-lift drag coefficient

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
        let (p, q, r) = (state[3], state[4], state[5]);

        // Define vectors
        let V_b = nalgebra::Vector3::new(u, v, w);
        let wbe_b = nalgebra::Vector3::new(p, q, r);

        let d_a = control_input[0]; // d_A (aileron)
        let d_e = control_input[1]; // d_e (elevator)
        let d_r = control_input[2]; // d_R (rudder)
        let d_th1 = control_input[3]; // d_th1 (throttle 1)
        let d_th2 = control_input[4]; // d_th2 (throttle 2)

        // ---------------INTERMEDIATE VARIABLES------------------------
        // Calculate airspeed
        let V_a = V_b.norm();

        // Calculate alpha and beta
        let alpha = w.atan2(u);
        let beta = (v / V_a).asin();

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

        let q_hat = q * CBAR / (2.0 * V_a);
        let Cm = CM0 + CMA * alpha + CMQ * q_hat;
        let M_sf = Cm * dynamic_pressure * S * CBAR;
        let M_bf = M_sf;

        let damping = CMQ * q;
        // tracing::info!("Damping: {:.5} {:.5}", q.to_degrees(), damping);

        // Calculate the CL_wb
        // let cl_wb = if alpha <= ALPHA_SWITCH {
        //     N * (alpha - ALPHA_L0)
        // } else {
        //     A3 * alpha.powi(3) + A2 * alpha.powi(2) + A1 * alpha + A0
        // };
        //
        // // Calculate CL_t
        // let epsilon = DEPSDA * (alpha - ALPHA_L0);
        // let alpha_t = alpha - epsilon + d_e + 1.3 * q * LT / va;
        // let cl_t = 3.1 * (S_TAIL / S) * alpha_t;
        //
        // // Total lift force
        // let cl = cl_wb + cl_t;
        //
        // // Total drag force (neglecting tail)
        // let cd = 0.13 + 0.07 * (5.5 * alpha + 0.654).powi(2);
        //
        // // Calculate sideforce
        // let cy = -1.6 * beta + 0.24 * d_r;
        //
        // // --------------DIMENSIONAL AERODYNAMIC FORCES---------------------
        // // Calculate the actual dimensional forces.  These are in F_s (stability axis)
        // let fa_s = nalgebra::Vector3::new(
        //     -cd * dynamic_pressure * S,
        //     cy * dynamic_pressure * S,
        //     -cl * dynamic_pressure * S,
        // );
        //
        // // Rotate these forces to F_b (body axis)
        // let r_bs = nalgebra::Matrix3::new(
        //     alpha.cos(),
        //     0.0,
        //     -(alpha.sin()),
        //     0.0,
        //     1.0,
        //     0.0,
        //     alpha.sin(),
        //     0.0,
        //     alpha.cos(),
        // );
        // let fa_b = r_bs * fa_s;
        //
        // // --------------AERODYNAMIC MOMENT ABOUT AC-------------------
        // // Calculate the moments in Fb.  Define eta, dCMdx and dCMdu
        // let eta11 = -1.4 * beta;
        // let eta21 = -0.59 - (3.1 * (S_TAIL * LT) / (S * CBAR)) * (alpha - epsilon);
        // let eta31 = (1.0 - alpha * (180.0 / (15.0 * PI))) * beta;
        //
        // let eta = nalgebra::Vector3::new(eta11, eta21, eta31);
        //
        // let d_cm_dx = (CBAR / va)
        //     * nalgebra::Matrix3::new(
        //         -11.0,
        //         0.0,
        //         5.0,
        //         0.0,
        //         -4.03 * (S_TAIL * LT.powi(2)) / (S * CBAR.powi(2)),
        //         0.0,
        //         1.7,
        //         0.0,
        //         -11.5,
        //     );
        //
        // let d_cm_du = nalgebra::Matrix3::new(
        //     -0.6,
        //     0.0,
        //     0.22,
        //     0.0,
        //     -3.1 * (S_TAIL * LT) / (S * CBAR),
        //     0.0,
        //     0.0,
        //     0.0,
        //     -0.63,
        // );
        //
        // // Now calculate CM = [Cl;Cm;Cn] about Aerodynamic center in Fb
        // let cm_ac_b = eta + d_cm_dx * wbe_b + d_cm_du * nalgebra::Vector3::new(d_a, d_e, d_r);
        //
        // // Normalize to an aerodynamic moment
        // let m_ac_b = cm_ac_b * dynamic_pressure * S * CBAR;
        //
        // // OPTIONAL: Covert this to stability axis
        // let r_sb = r_bs.transpose();
        // let m_ac_s = r_sb * m_ac_b;
        //
        // // --------------AERODYNAMIC MOMENT ABOUT CG-------------------
        // // Transfer moment to cg
        // let rcg_b = nalgebra::Vector3::new(X_CG, Y_CG, Z_CG);
        // let rac_b = nalgebra::Vector3::new(X_AC, Y_AC, Z_AC);
        // let ma_cg_b = r_bs * m_ac_s + fa_b.cross(&(rcg_b - rac_b));
        //
        // // -----------------ENGINE FORCE & MOMENT----------------------------
        // // Now effect of engine.  First, calculate the thrust of each engine
        // let f1 = d_th1 * U_MAX;
        // let f2 = d_th2 * U_MAX;
        //
        // // Assuming that engine thrust is aligned with Fb, we have
        // let fe1_b = nalgebra::Vector3::new(f1, 0.0, 0.0);
        // let fe2_b = nalgebra::Vector3::new(f2, 0.0, 0.0);
        //
        // let fe_b = fe1_b + fe2_b;
        //
        // // Now engine moment due to offset of engine thrust from CoG.
        // // Note: This is weird because the sign of y-axis seems to be flipped
        // let mew1 = nalgebra::Vector3::new(X_CG - X_APT1, Y_APT1 - Y_CG, Z_CG - Z_APT1);
        //
        // let me_cg1_b = mew1.cross(&fe1_b);
        //
        // let me_cg_b = me_cg1_b;

        let g_ned = nalgebra::Vector3::new(0.0, 0.0, G);
        let g_bf = rotation_matrix.transpose() * g_ned;
        let Fg_bf = self.mass() * g_bf;

        // tracing::info!(
        //     "gravity: {}\tlift: {}\talpha: {}",
        //     fg_b.norm(),
        //     cl_wb,
        //     alpha.to_degrees()
        // );

        // let f_b = fg_b + fe_b + fa_b;
        // let m_cg_b = ma_cg_b + me_cg_b;
        let F_bf = Fg_bf + L_bf + D_bf;
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
