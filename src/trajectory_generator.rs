use crate::data::Data;
use noise::{NoiseFn, Perlin};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub struct TrajectoryGenerator {
    data_handle: Arc<Mutex<Data>>,
    shutdown_trigger: Arc<AtomicBool>,
}

impl TrajectoryGenerator {
    fn new(
        data_handle: Arc<Mutex<Data>>,
        shutdown_trigger: Arc<AtomicBool>,
    ) -> TrajectoryGenerator {
        TrajectoryGenerator {
            data_handle,
            shutdown_trigger,
        }
    }

    fn generate_data() -> Data {
        let perlin = Perlin::new(2137); // replace with static version?

        let secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards")
            .as_secs_f64();

        let x = upscale(perlin.get([secs, 0.0, 0.0]));
        let y = upscale(perlin.get([0.0, secs, 0.0]));
        let z = upscale(perlin.get([0.0, 0.0, secs]));

        Data {
            x,
            y,
            z,
            timestamp: SystemTime::now(),
        }
    }

    pub fn run(
        period: f64,
        shutdown_trigger: Arc<AtomicBool>,
    ) -> (Arc<Mutex<Data>>, JoinHandle<()>) {
        let data_handle = Arc::new(Mutex::new(Data::new()));
        let generator =
            TrajectoryGenerator::new(Arc::clone(&data_handle), Arc::clone(&shutdown_trigger));

        let generator_handle = std::thread::spawn(move || {
            while !generator.shutdown_trigger.load(Ordering::SeqCst) {
                {
                    *generator.data_handle.lock().unwrap() = TrajectoryGenerator::generate_data();
                }

                std::thread::sleep(Duration::from_secs_f64(period));
            }
            println!("Trajectory generator removed");
        });

        (Arc::clone(&data_handle), generator_handle)
    }
}

#[inline]
fn upscale(perlin_value: f64) -> f64 {
    (perlin_value + 1.0) * 50.0
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
        let (data_handle, handle) = TrajectoryGenerator::run(0.1, Arc::clone(&shutdown));

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
        let (data_handle, handle) = TrajectoryGenerator::run(0.05, Arc::clone(&shutdown));

        std::thread::sleep(Duration::from_millis(100));
        shutdown.store(true, Ordering::SeqCst);

        let old_data = { *data_handle.lock().unwrap() };

        handle.join().unwrap();

        std::thread::sleep(Duration::from_millis(100));

        let new_data = *data_handle.lock().unwrap();

        assert_eq!(old_data.timestamp, new_data.timestamp);
    }
}
