use crate::data::Data;
use noise::{NoiseFn, Perlin};
use rand::Rng;
use std::{
    f64::consts::PI,
    num::NonZeroU32,
    sync::atomic::{AtomicBool, Ordering},
    sync::{Arc, Mutex},
    thread::JoinHandle,
    time::{SystemTime, UNIX_EPOCH},
};

use crate::utils::get_cycle_duration;

const HELIX_FREQUENCY: f64 = 0.4;

#[derive(Clone, Copy)]
pub enum GenerationMode {
    Random,
    Perlin,
    DeterminicticPerlin,
    AngledHelical(f64),
}

pub struct TrajectoryGenerator {
    data_handle: Arc<Mutex<Data>>,
    shutdown_trigger: Arc<AtomicBool>,
    mode: GenerationMode,
    seed: u32,
    step: f64,
}

impl TrajectoryGenerator {
    fn new(
        data_handle: Arc<Mutex<Data>>,
        shutdown_trigger: Arc<AtomicBool>,
        mode: GenerationMode,
        seed: u32,
    ) -> TrajectoryGenerator {
        TrajectoryGenerator {
            data_handle,
            shutdown_trigger,
            mode,
            seed,
            step: 0.0,
        }
    }

    fn generate_data(&mut self) -> Data {
        match self.mode {
            GenerationMode::Random => self.generate_rnd_data(),
            GenerationMode::Perlin => self.generate_perlin_data(),
            GenerationMode::DeterminicticPerlin => self.generate_deterministic_perlin_data(),
            GenerationMode::AngledHelical(step) => self.generate_angled_helical_data(step),
        }
    }

    fn generate_perlin_data(&self) -> Data {
        let perlin = Perlin::new(self.seed);

        let secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards")
            .as_secs_f64();

        let x = upscale(perlin.get([secs, 0.0, 0.0]));
        let y = upscale(perlin.get([0.0, secs, 0.0]));
        let z = upscale(perlin.get([0.0, 0.0, secs]));

        println!("Data: {x},\t {y},\t {z}");
        Data {
            x,
            y,
            z,
            timestamp: SystemTime::now(),
        }
    }

    fn generate_deterministic_perlin_data(&mut self) -> Data {
        let perlin = Perlin::new(self.seed);
        self.step += 0.1;

        let x = upscale(perlin.get([self.step, 0.0, 0.0]));
        let y = upscale(perlin.get([0.0, self.step, 0.0]));
        let z = upscale(perlin.get([0.0, 0.0, self.step]));

        println!("Data: {x},\t {y},\t {z}");
        Data {
            x,
            y,
            z,
            timestamp: SystemTime::now(),
        }
    }

    fn generate_rnd_data(&self) -> Data {
        let mut rng = rand::rng();
        Data {
            x: rng.random_range(0.0..=100.0),
            y: rng.random_range(0.0..=100.0),
            z: rng.random_range(0.0..=100.0),
            timestamp: SystemTime::now(),
        }
    }

    fn generate_angled_helical_data(&mut self, step_size: f64) -> Data {
        self.step += step_size;
        Data {
            x: upscale(self.step.sin()),
            y: upscale(self.step.cos()),
            z: upscale(self.step.sin()),
            timestamp: SystemTime::now(),
        }
    }
}

#[inline]
fn upscale(value: f64) -> f64 {
    (value + 1.0) * 50.0
}

#[inline]
fn get_helix_step(gen_freq: u32) -> f64 {
    2.0 * PI * (HELIX_FREQUENCY / (gen_freq as f64))
}

pub struct TrajectoryGeneratorBuilder {
    mode: GenerationMode,
    frequency: NonZeroU32,
    seed: Option<u32>,
}

#[allow(dead_code)]
impl TrajectoryGeneratorBuilder {
    pub fn new() -> Self {
        TrajectoryGeneratorBuilder {
            mode: GenerationMode::Random,
            frequency: NonZeroU32::new(1).unwrap(),
            seed: None,
        }
    }

    pub fn with_perlin_mode(mut self) -> Self {
        self.mode = GenerationMode::Perlin;
        self
    }

    pub fn with_determinisitic_perlin_mode(mut self) -> Self {
        self.mode = GenerationMode::DeterminicticPerlin;
        self
    }

    pub fn with_random_mode(mut self) -> Self {
        self.mode = GenerationMode::Random;
        self
    }

    // make sure to delcare frequency before this mode
    pub fn with_angled_helical_mode(mut self) -> Self {
        self.mode = GenerationMode::AngledHelical(get_helix_step(self.frequency.get()));
        self
    }

    pub fn with_frequency(mut self, frequency: NonZeroU32) -> Self {
        self.frequency = frequency;
        self
    }

    pub fn with_seed(mut self, seed: u32) -> Self {
        self.seed = Some(seed);
        self
    }

    pub fn with_yellow_seed(mut self) -> Self {
        self.seed = Some(0x859);
        self
    }

    pub fn spawn(&self, shutdown_trigger: Arc<AtomicBool>) -> (Arc<Mutex<Data>>, JoinHandle<()>) {
        let data_handle = Arc::new(Mutex::new(Data::new()));
        let mut generator = TrajectoryGenerator::new(
            Arc::clone(&data_handle),
            Arc::clone(&shutdown_trigger),
            self.mode,
            self.seed.unwrap_or_default(),
        );

        *generator.data_handle.lock().unwrap() = generator.generate_data();

        let frequency = self.frequency;
        let generator_handle = std::thread::spawn(move || {
            while !generator.shutdown_trigger.load(Ordering::SeqCst) {
                {
                    *generator.data_handle.lock().unwrap() = generator.generate_data();
                }

                std::thread::sleep(get_cycle_duration(frequency));
            }
            println!("Trajectory generator removed");
        });

        (Arc::clone(&data_handle), generator_handle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;
    use std::sync::Arc;
    use std::time::Duration;

    #[test]
    fn test_rnd_trajectory_generator_updates_data() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (data_handle, handle) = TrajectoryGeneratorBuilder::new()
            .with_frequency(NonZeroU32::new(10).unwrap())
            .with_random_mode()
            .spawn(Arc::clone(&shutdown));

        std::thread::sleep(Duration::from_millis(300));

        shutdown.store(true, Ordering::SeqCst);
        handle.join().unwrap();

        let data = data_handle.lock().unwrap();
        assert!(data.x >= 0.0 && data.x <= 100.0);
        assert!(data.y >= 0.0 && data.y <= 100.0);
        assert!(data.z >= 0.0 && data.z <= 100.0);
    }

    #[test]
    fn test_angled_helical_trajectory_generator_updates_data() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (data_handle, handle) = TrajectoryGeneratorBuilder::new()
            .with_frequency(NonZeroU32::new(5).unwrap())
            .with_angled_helical_mode()
            .spawn(Arc::clone(&shutdown));

        std::thread::sleep(Duration::from_millis(300));

        shutdown.store(true, Ordering::SeqCst);
        handle.join().unwrap();

        let data = data_handle.lock().unwrap();
        assert!(data.x >= 0.0 && data.x <= 100.0);
        assert!(data.y >= 0.0 && data.y <= 100.0);
        assert!(data.z >= 0.0 && data.z <= 100.0);
    }

    #[test]
    fn test_perlin_trajectory_generator_updates_data() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (data_handle, handle) = TrajectoryGeneratorBuilder::new()
            .with_frequency(NonZeroU32::new(5).unwrap())
            .with_perlin_mode()
            .spawn(Arc::clone(&shutdown));

        std::thread::sleep(Duration::from_millis(300));

        shutdown.store(true, Ordering::SeqCst);
        handle.join().unwrap();

        let data = data_handle.lock().unwrap();
        assert!(data.x >= 0.0 && data.x <= 100.0);
        assert!(data.y >= 0.0 && data.y <= 100.0);
        assert!(data.z >= 0.0 && data.z <= 100.0);
    }

    #[test]
    fn test_deterministic_perlin_trajectory_generator_updates_data() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (data_handle, handle) = TrajectoryGeneratorBuilder::new()
            .with_frequency(NonZeroU32::new(5).unwrap())
            .with_determinisitic_perlin_mode()
            .spawn(Arc::clone(&shutdown));

        std::thread::sleep(Duration::from_millis(300));

        shutdown.store(true, Ordering::SeqCst);
        handle.join().unwrap();

        let data = data_handle.lock().unwrap();
        assert!(data.x >= 0.0 && data.x <= 100.0);
        assert!(data.y >= 0.0 && data.y <= 100.0);
        assert!(data.z >= 0.0 && data.z <= 100.0);
    }

    #[test]
    fn test_shutdown_trigger_stops_generation() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (data_handle, handle) = TrajectoryGeneratorBuilder::new()
            .with_yellow_seed()
            .with_frequency(NonZeroU32::new(5).unwrap())
            .with_perlin_mode()
            .spawn(Arc::clone(&shutdown));

        std::thread::sleep(Duration::from_millis(100));
        shutdown.store(true, Ordering::SeqCst);
        let old_data = { *data_handle.lock().unwrap() };
        handle.join().unwrap();

        std::thread::sleep(Duration::from_millis(100));

        let new_data = *data_handle.lock().unwrap();

        assert_eq!(old_data.timestamp, new_data.timestamp);
    }
}
