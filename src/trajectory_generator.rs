use crate::data::Data;
use noise::{NoiseFn, Perlin};
use rand::Rng;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(Clone, Copy)]
pub enum GenerationMode {
    Random,
    Perlin,
}

pub struct TrajectoryGenerator {
    data_handle: Arc<Mutex<Data>>,
    shutdown_trigger: Arc<AtomicBool>,
    mode: GenerationMode,
    seed: u32,
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
        }
    }

    fn generate_data(&self) -> Data {
        match self.mode {
            GenerationMode::Random => self.generate_rnd_data(),
            GenerationMode::Perlin => self.generate_perlin_data(),
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

    fn generate_rnd_data(&self) -> Data {
        let mut rng = rand::rng();
        Data {
            x: rng.random_range(0.0..=100.0),
            y: rng.random_range(0.0..=100.0),
            z: rng.random_range(0.0..=100.0),
            timestamp: SystemTime::now(),
        }
    }
}

#[inline]
fn upscale(perlin_value: f64) -> f64 {
    (perlin_value + 1.0) * 50.0
}

pub struct TrajectoryGeneratorBuilder {
    mode: GenerationMode,
    period: f64,
    seed: Option<u32>,
}

#[allow(dead_code)]
impl TrajectoryGeneratorBuilder {
    pub fn new() -> Self {
        TrajectoryGeneratorBuilder {
            mode: GenerationMode::Random,
            period: 1.0,
            seed: None,
        }
    }

    pub fn with_perlin_mode(mut self) -> Self {
        self.mode = GenerationMode::Perlin;
        self
    }

    pub fn with_random_mode(mut self) -> Self {
        self.mode = GenerationMode::Random;
        self
    }

    pub fn with_period(mut self, period: f64) -> Self {
        self.period = period;
        self
    }
    pub fn with_seed(mut self, seed: u32) -> Self {
        self.seed = Some(seed);
        self
    }

    pub fn spawn(&self, shutdown_trigger: Arc<AtomicBool>) -> (Arc<Mutex<Data>>, JoinHandle<()>) {
        let data_handle = Arc::new(Mutex::new(Data::new()));
        let generator = TrajectoryGenerator::new(
            Arc::clone(&data_handle),
            Arc::clone(&shutdown_trigger),
            self.mode,
            self.seed.unwrap_or_default(),
        );

        *generator.data_handle.lock().unwrap() = generator.generate_data();

        let period = self.period;
        let generator_handle = std::thread::spawn(move || {
            while !generator.shutdown_trigger.load(Ordering::SeqCst) {
                {
                    *generator.data_handle.lock().unwrap() = generator.generate_data();
                }

                std::thread::sleep(Duration::from_secs_f64(period));
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
    fn test_trajectory_generator_updates_data() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (data_handle, handle) = TrajectoryGeneratorBuilder::new()
            .with_period(0.1)
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
    fn test_shutdown_trigger_stops_generation() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (data_handle, handle) = TrajectoryGeneratorBuilder::new()
            .with_period(0.133)
            .with_perlin_mode()
            .with_seed(1000)
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
