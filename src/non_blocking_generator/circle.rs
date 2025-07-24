use super::PositionGenerator;
use crate::data::Data;
use std::{
    f64::consts::PI,
    time::{Instant, SystemTime},
};

const RADIUS: f64 = 30.0;
const FREQUENCY: f64 = 0.7;
const X_START: f64 = RADIUS * 2.0;
const Y_START: f64 = RADIUS * 2.0;

#[derive(Clone, Copy)]
pub struct CircleGenerator {
    start: Instant,
}

unsafe impl Send for CircleGenerator {}

impl CircleGenerator {
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl PositionGenerator for CircleGenerator {
    fn get_position(&self) -> Data {
        let delta_time = self.start.elapsed().as_secs_f64();
        let position = 2.0 * PI * delta_time * FREQUENCY;
        let x = X_START + RADIUS * position.cos();
        let y = Y_START + RADIUS * position.sin();
        let z = 0.0;
        Data {
            x,
            y,
            z,
            timestamp: SystemTime::now(),
        }
    }
}
