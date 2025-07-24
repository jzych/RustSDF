use super::*;
use crate::data::Data;
use std::{
    f64::consts::PI,
    time::{Instant, SystemTime},
};

const HELIX_FREQUENCY: f64 = 0.5;

#[derive(Clone, Copy)]
pub struct HelixGenerator {
    start_time: Instant,
}

unsafe impl Send for HelixGenerator {}

impl HelixGenerator {
    pub fn new() -> Self {
        Self {
            start_time: Instant::now(),
        }
    }
}

impl PositionGenerator for HelixGenerator {
    fn get_position(&self) -> Data {
        let delta_time = self.start_time.elapsed().as_secs_f64();
        let step = HELIX_FREQUENCY * delta_time * 2.0 * PI;
        Data {
            x: upscale(step.sin()),
            y: upscale(step.cos()),
            z: upscale(step.sin()),
            timestamp: SystemTime::now(),
        }
    }
}
