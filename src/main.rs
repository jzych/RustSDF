use rand::Rng;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime};

use crate::data::Data;
use crate::trajectory_generator::TrajectoryGenerator;
use crate::input_generator::Imu;

pub mod data;
mod input_generator;
mod trajectory_generator;

fn generate_rnd_data() -> Data {
    let mut rng = rand::rng();
    Data {
        x: rng.random_range(0.0..=100.0),
        y: rng.random_range(0.0..=100.0),
        z: rng.random_range(0.0..=100.0),
        timestamp: SystemTime::now(),
    }
}

fn create_data_source(
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>>,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<()> {
    thread::spawn(move || {
        while !shutdown.load(Ordering::SeqCst) {
            let output_data = generate_rnd_data();

            let registry = consumer_registry.lock().unwrap();
            for client in registry.values() {
                let _ = client.send(output_data);
            }

            thread::sleep(Duration::from_secs(1));
        }
        println!("Producer removed");
    })
}

fn register_new_consumer(
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>>,
) -> (usize, mpsc::Receiver<Data>) {
    let (input_tx, input_rx) = mpsc::channel();
    let mut registry = consumer_registry.lock().unwrap();
    let id = registry.len();

    registry.insert(id, input_tx);

    (id, input_rx)
}

fn deregister_consumer(
    id: usize,
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>>,
) {
    let mut registry = consumer_registry.lock().unwrap();
    registry.remove(&id);
    println!("Consumer{id} removed");
}

fn create_data_consumer(
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>>,
) -> JoinHandle<()> {
    let (id, input_rx) = register_new_consumer(Arc::clone(&consumer_registry));

    let handle = thread::spawn(move || {
        for data in input_rx {
            println!(
                "Consumer{}: received: {}, {}, {}",
                id, data.x, data.y, data.z
            );
        }

        deregister_consumer(id, consumer_registry);
    });
    handle
}

fn system_shutdown(
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>>,
    shutdown_trigger: Arc<AtomicBool>,
) {
    shutdown_trigger.store(true, Ordering::SeqCst);
    let mut registry = consumer_registry.lock().unwrap();
    registry.clear();
}

fn main() {
    println!("Hello RustSDF!");
    let position_generator = Arc::new(Mutex::new(TrajectoryGenerator));
    let (tx, rx) = mpsc::channel();
    let imu = Imu::run(Arc::clone(&position_generator), tx);

    let consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>> =
        Arc::new(Mutex::new(HashMap::new()));
    let shutdown_trigger = Arc::new(AtomicBool::new(false));

    let consumer0_handle = create_data_consumer(Arc::clone(&consumer_registry));
    let consumer1_handle = create_data_consumer(Arc::clone(&consumer_registry));
    let data_source_handle = create_data_source(
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
    
    drop(rx);
    imu.join().unwrap();
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;

    use super::*;

    #[test]
    fn test_register_and_deregister_of_consumer() {
        let consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>> =
            Arc::new(Mutex::new(HashMap::new()));

        let (id, _rx) = register_new_consumer(Arc::clone(&consumer_registry));
        assert!(consumer_registry.lock().unwrap().contains_key(&id));

        deregister_consumer(id, Arc::clone(&consumer_registry));
        assert!(!consumer_registry.lock().unwrap().contains_key(&id));
    }

    #[test]
    fn test_producer_sends_data() {
        let consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>> =
            Arc::new(Mutex::new(HashMap::new()));
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (_id, rx) = register_new_consumer(Arc::clone(&consumer_registry));

        let test_producer_handle = create_data_source(
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
