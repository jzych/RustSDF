use super::*;
use crate::data::Data;
use noise::{NoiseFn, Perlin};
use std::time::{Instant, SystemTime};

#[derive(Clone, Copy)]
pub struct PerlinGenerator {
    start_time: Instant,
    perlin_noise_generator: Perlin,
}

unsafe impl Send for PerlinGenerator {}

impl PerlinGenerator {
    pub fn new(seed: u32) -> Self {
        Self {
            start_time: Instant::now(),
            perlin_noise_generator: Perlin::new(seed),
        }
    }
}

impl PositionGenerator for PerlinGenerator {
    fn get_position(&self) -> Data {
        let delta_time = self.start_time.elapsed().as_secs_f64();

        let x = upscale(self.perlin_noise_generator.get([delta_time, 0.0, 0.0]));
        let y = upscale(self.perlin_noise_generator.get([0.0, delta_time, 0.0]));
        let z = upscale(self.perlin_noise_generator.get([0.0, 0.0, delta_time]));
        Data {
            x,
            y,
            z,
            timestamp: SystemTime::now(),
        }
    }
}
