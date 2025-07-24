mod angled_helical;
mod circle;
mod perlin;

use dyn_clone::DynClone;
use crate::data::Data;
use {angled_helical::HelixGenerator, circle::CircleGenerator, perlin::PerlinGenerator};

pub trait PositionGenerator: DynClone + Send {
    fn get_position(&self) -> Data;
}

#[inline]
fn upscale(value: f64) -> f64 {
    (value + 2.0) * 50.0
}

pub enum PositionGeneratorType {
    Perlin,
    Circle,
    AngledHelical,
}

pub fn get_position_generator(generator_type: PositionGeneratorType) -> Box<dyn PositionGenerator> {
    match generator_type {
        PositionGeneratorType::Perlin => Box::new(PerlinGenerator::new(3232)),
        PositionGeneratorType::AngledHelical => Box::new(HelixGenerator::new()),
        PositionGeneratorType::Circle => Box::new(CircleGenerator::new()),
    }
}
