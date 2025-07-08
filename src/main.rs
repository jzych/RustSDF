use rand::Rng;
use std::collections::HashMap;
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
) -> JoinHandle<()> {
    thread::spawn(move || {
        for _ in 0..5 {
            let output_data = generate_rnd_data();

            let registry = consumer_registry.lock().unwrap();
            for client in registry.values() {
                let _ = client.send(output_data);
            }

            thread::sleep(Duration::from_secs(2));
        }
    })
}

fn register_new_consumer(
    channel: mpsc::Sender<Data>,
    consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>>,
) -> usize {
    let mut registry = consumer_registry.lock().unwrap();
    let id = registry.len();

    registry.insert(id, channel);

    id
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
    let (input_tx, input_rx) = mpsc::channel();
    let id = register_new_consumer(input_tx, Arc::clone(&consumer_registry));

    let handle = thread::spawn(move || {
        let mut counter = 0;

        for data in &input_rx {
            counter += 1;
            println!(
                "Consumer{}: received: {}, {}, {}",
                id, data.x, data.y, data.z
            );
            if counter > 4 {
                break;
            }
        }

        deregister_consumer(id, consumer_registry);
    });
    handle
}

fn main() {
    println!("Hello RustSDF!");
    let position_generator = Arc::new(Mutex::new(TrajectoryGenerator));
    let (tx, rx) = mpsc::channel();
    let imu = Imu::run(Arc::clone(&position_generator), tx);

    let consumer_registry: Arc<Mutex<HashMap<usize, mpsc::Sender<Data>>>> =
        Arc::new(Mutex::new(HashMap::new()));

    let consumer0_handle = create_data_consumer(Arc::clone(&consumer_registry));
    let consumer1_handle = create_data_consumer(Arc::clone(&consumer_registry));
    let data_source_handle = create_data_source(Arc::clone(&consumer_registry));

    data_source_handle.join().unwrap();
    consumer1_handle.join().unwrap();
    consumer0_handle.join().unwrap();
    
    drop(rx);
    imu.join().unwrap();
}

#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
