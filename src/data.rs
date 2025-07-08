use std::time::SystemTime;

#[derive(Debug, Copy, Clone)]
pub struct Data {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub timestamp: SystemTime,
}

impl Data {
    pub fn new() -> Self {
        Data {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            timestamp: SystemTime::now(),
        }
    }
}

impl Default for Data {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Copy, Clone)]
pub enum Telemetry {
    Acceleration(Data),
    Position(Data),
}
