use nalgebra::Vector3;
use std::ops::{Deref, DerefMut};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NED(Vector3<f64>);

impl NED {
    pub fn new(north: f64, east: f64, down: f64) -> Self {
        Self(Vector3::new(north, east, down))
    }

    pub fn north(&self) -> f64 {
        self.0.x
    }

    pub fn east(&self) -> f64 {
        self.0.y
    }

    pub fn down(&self) -> f64 {
        self.0.z
    }

    pub fn set_north(&mut self, val: f64) {
        self.0.x = val;
    }

    pub fn set_east(&mut self, val: f64) {
        self.0.y = val;
    }

    pub fn set_down(&mut self, val: f64) {
        self.0.z = val;
    }
}

// Allow use as Vector3<f64>
impl Deref for NED {
    type Target = Vector3<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for NED {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LLA(Vector3<f64>);

impl LLA {
    pub fn new(lat: f64, lon: f64, alt: f64) -> Self {
        Self(Vector3::new(lat, lon, alt))
    }

    pub fn lat(&self) -> f64 {
        self.0.x
    }

    pub fn lon(&self) -> f64 {
        self.0.y
    }

    pub fn alt(&self) -> f64 {
        self.0.z
    }

    pub fn set_lat(&mut self, val: f64) {
        self.0.x = val;
    }

    pub fn set_lon(&mut self, val: f64) {
        self.0.y = val;
    }

    pub fn set_alt(&mut self, val: f64) {
        self.0.z = val;
    }
}

// Allow use as Vector3<f64>
impl Deref for LLA {
    type Target = Vector3<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for LLA {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RPY(Vector3<f64>);

impl RPY {
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self(Vector3::new(roll, pitch, yaw))
    }

    pub fn roll(&self) -> f64 {
        self.0.x
    }

    pub fn pitch(&self) -> f64 {
        self.0.y
    }

    pub fn yaw(&self) -> f64 {
        self.0.z
    }

    pub fn set_roll(&mut self, val: f64) {
        self.0.x = val;
    }

    pub fn set_pitch(&mut self, val: f64) {
        self.0.y = val;
    }

    pub fn set_yaw(&mut self, val: f64) {
        self.0.z = val;
    }
}

impl Deref for RPY {
    type Target = Vector3<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for RPY {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Degrees(pub f64);

#[derive(Debug, Clone, Copy)]
pub struct Radians(pub f64);

pub trait IntoRadians {
    fn to_radians(self) -> f64;
}

impl IntoRadians for Degrees {
    fn to_radians(self) -> f64 {
        self.0.to_radians()
    }
}

impl IntoRadians for Radians {
    fn to_radians(self) -> f64 {
        self.0
    }
}

pub trait AngleExt {
    fn degrees(self) -> Degrees;
    fn radians(self) -> Radians;
}
impl AngleExt for f64 {
    fn degrees(self) -> Degrees {
        Degrees(self)
    }

    fn radians(self) -> Radians {
        Radians(self)
    }
}