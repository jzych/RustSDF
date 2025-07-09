use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crate::data::Telemetry;
use crate::imu::Imu;
use crate::trajectory_generator::TrajectoryGenerator;

pub mod data;
mod imu;
mod trajectory_generator;

fn create_data_source(
    trajectory_generator: Arc<Mutex<TrajectoryGenerator>>,
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>>,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<()> {
    let consumer_registry = consumer_registry.lock().unwrap();
    Imu::run(
        trajectory_generator,
        consumer_registry.values().cloned().collect(),
        Arc::clone(&shutdown),
    )
}

fn register_new_consumer(
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>>,
) -> (usize, mpsc::Receiver<Telemetry>) {
    let (input_tx, input_rx) = mpsc::channel();
    let mut registry = consumer_registry.lock().unwrap();
    let id = registry.len();

    registry.insert(id, input_tx);

    (id, input_rx)
}

fn deregister_consumer(
    id: usize,
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>>,
) {
    let mut registry = consumer_registry.lock().unwrap();
    registry.remove(&id);
    println!("Consumer{id} removed");
}

fn create_data_consumer(
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>>,
) -> JoinHandle<()> {
    let (id, input_rx) = register_new_consumer(Arc::clone(&consumer_registry));

    let handle = thread::spawn(move || {
        for data in input_rx {
            match data {
                Telemetry::Acceleration(d) => {
                    println!("Consumer{}: received: {}, {}, {}", id, d.x, d.y, d.z)
                }
                Telemetry::Position(d) => {
                    println!("Consumer{}: received: {}, {}, {}", id, d.x, d.y, d.z)
                }
            }
        }

        deregister_consumer(id, consumer_registry);
    });
    handle
}

fn system_shutdown(
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>>,
    shutdown_trigger: Arc<AtomicBool>,
) {
    shutdown_trigger.store(true, Ordering::SeqCst);
    let mut registry = consumer_registry.lock().unwrap();
    registry.clear();
}

fn main() {
    println!("Hello RustSDF!");
    let position_generator = Arc::new(Mutex::new(TrajectoryGenerator));
    let consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>> =
        Arc::new(Mutex::new(HashMap::new()));
    let shutdown_trigger = Arc::new(AtomicBool::new(false));

    let consumer0_handle = create_data_consumer(Arc::clone(&consumer_registry));
    let consumer1_handle = create_data_consumer(Arc::clone(&consumer_registry));
    let data_source_handle = create_data_source(
        Arc::clone(&position_generator),
        Arc::clone(&consumer_registry),
        Arc::clone(&shutdown_trigger),
    );

    thread::sleep(Duration::from_secs(6));
    system_shutdown(
        Arc::clone(&consumer_registry),
        Arc::clone(&shutdown_trigger),
    );

    data_source_handle.join().unwrap();
    consumer1_handle.join().unwrap();
    consumer0_handle.join().unwrap();
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;

    use super::*;

    #[test]
    fn test_register_and_deregister_of_consumer() {
        let consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>> =
            Arc::new(Mutex::new(HashMap::new()));

        let (id, _rx) = register_new_consumer(Arc::clone(&consumer_registry));
        assert!(consumer_registry.lock().unwrap().contains_key(&id));

        deregister_consumer(id, Arc::clone(&consumer_registry));
        assert!(!consumer_registry.lock().unwrap().contains_key(&id));
    }

    #[test]
    fn test_producer_sends_data() {
        let consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Telemetry>>>> =
            Arc::new(Mutex::new(HashMap::new()));
        let position_generator = Arc::new(Mutex::new(TrajectoryGenerator));
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (_id, rx) = register_new_consumer(Arc::clone(&consumer_registry));

        let test_producer_handle = create_data_source(
            Arc::clone(&position_generator),
            Arc::clone(&consumer_registry),
            Arc::clone(&shutdown_trigger),
        );

        thread::sleep(Duration::from_secs(2));
        system_shutdown(
            Arc::clone(&consumer_registry),
            Arc::clone(&shutdown_trigger),
        );
        test_producer_handle.join().unwrap();

        let received: Vec<_> = rx.try_iter().collect();
        assert!(!received.is_empty());
    }
}
