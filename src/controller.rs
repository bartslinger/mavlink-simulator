use crate::fixed_wing::ControlInput;
use crate::flight_dynamics::State;

pub struct ControlSetpoint {
    pub altitude: f64,
    pub airspeed: f64,
}
pub struct Controller {
    throttle_integrator: f64,
    pitch_integrator: f64,
    altitude_integrator: f64,
    test: f64,
}

impl Controller {
    pub fn new() -> Self {
        Self {
            throttle_integrator: 0.0,
            pitch_integrator: 0.0,
            altitude_integrator: 0.0,
            test: 0.6,
        }
    }

    pub fn calculate_control_input(
        &mut self,
        state: &State,
        setpoint: ControlSetpoint,
        dt: f64,
    ) -> ControlInput {
        let (_roll, pitch, _yaw) = rpy(state);

        let velocity = nalgebra::Vector3::new(state[0], state[1], state[2]);
        let airspeed = velocity.norm();
        let airspeed_error = setpoint.airspeed - airspeed;
        self.throttle_integrator += 0.05 * airspeed_error * dt;
        let throttle = 0.01 * airspeed_error + self.throttle_integrator;

        let altitude = -state[12];

        let altitude_error = setpoint.altitude - altitude;
        self.altitude_integrator += 0.05 * altitude_error * dt;
        self.altitude_integrator = self.altitude_integrator.max(-5.0).min(5.0);

        let pitch_setpoint = (altitude_error * 0.7 + self.altitude_integrator);
        // constrain
        let pitch_setpoint = pitch_setpoint.max(-20.0).min(20.0).to_radians();

        // let pitch_setpoint = -2.0_f64.to_radians();
        let pitch_error = pitch_setpoint - pitch;
        let pitch_kp = 1.0;
        let pitchrate_setpoint = pitch_error * pitch_kp;

        let pitchrate_error = pitchrate_setpoint - state[4];
        let pitchrate_ki = 2.0;
        let pitchrate_kp = 2.0;
        self.pitch_integrator += pitchrate_ki * pitchrate_error * dt;
        let pitch = pitchrate_kp * pitchrate_error + self.pitch_integrator;

        self.test += dt;
        let pitch = if self.test > 0.5 { pitch } else { 0.35 };
        if self.test > 7.0 {
            self.test = 0.0;
        }

        // Placeholder for control input calculation
        ControlInput {
            roll: 0.0,
            pitch,
            yaw: 0.0,
            throttle,
        }
    }
}

pub fn rpy(state: &State) -> (f64, f64, f64) {
    let q = nalgebra::Quaternion::new(state[6], state[7], state[8], state[9]);
    nalgebra::UnitQuaternion::from_quaternion(q).euler_angles()
}
