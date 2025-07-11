use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crate::communication_registry::CommunicationRegistry;
use crate::communication_registry::DataSource;
use crate::data::{Data, Telemetry};
use crate::imu::Imu;
use crate::trajectory_generator::TrajectoryGenerator;

mod communication_registry;
pub mod data;
mod imu;
mod trajectory_generator;

//Refresh rate in Hz
const GENERATOR_FREQ: f64 = 10.0;

#[allow(unused)]
#[derive(Debug)]
enum Error {
    StartupError(&'static str),
}

fn start_imu(
    trajectory_data: Arc<Mutex<Data>>,
    communication_registry: &mut CommunicationRegistry,
    shutdown: Arc<AtomicBool>,
) -> Result<JoinHandle<()>, Error> {
    match communication_registry.get_registered_transmitters(DataSource::Imu) {
        Some(transmitters) => Ok(Imu::run(
            trajectory_data,
            transmitters,
            Arc::clone(&shutdown),
        )),
        None => Err(Error::StartupError(
            "No subscribers for Imu. Start aborted.",
        )),
    }
}

fn create_data_consumer(consumer_registry: &mut CommunicationRegistry) -> JoinHandle<()> {
    let (tx, input_rx) = mpsc::channel();
    consumer_registry.register_for_input(DataSource::Imu, tx);

    let handle = thread::spawn(move || {
        for data in input_rx {
            match data {
                Telemetry::Acceleration(d) => {
                    println!(
                        "Consumer{:?}: received: {}, {}, {}",
                        DataSource::Imu,
                        d.x,
                        d.y,
                        d.z
                    )
                }
                Telemetry::Position(d) => {
                    println!(
                        "Consumer{:?}: received: {}, {}, {}",
                        DataSource::Gps,
                        d.x,
                        d.y,
                        d.z
                    )
                }
            }
        }
        println!("Channel has been closed, exiting the thread.");
    });
    handle
}

fn system_shutdown(shutdown_trigger: Arc<AtomicBool>) {
    shutdown_trigger.store(true, Ordering::SeqCst);
}

fn main() -> Result<(), Error> {
    let mut communication_registry = CommunicationRegistry::new();
    let shutdown_trigger = Arc::new(AtomicBool::new(false));

    let consumer0_handle = create_data_consumer(&mut communication_registry);
    let consumer1_handle = create_data_consumer(&mut communication_registry);
    let (generated_data_handle, generator_handle) =
        TrajectoryGenerator::run(1.0 / GENERATOR_FREQ, Arc::clone(&shutdown_trigger));
    let data_source_handle = start_imu(
        Arc::clone(&generated_data_handle),
        &mut communication_registry,
        Arc::clone(&shutdown_trigger),
    )?;

    thread::sleep(Duration::from_secs(6));
    system_shutdown(Arc::clone(&shutdown_trigger));

    generator_handle.join().unwrap();
    data_source_handle.join().unwrap();
    consumer1_handle.join().unwrap();
    consumer0_handle.join().unwrap();
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;

    use super::*;

    #[test]
    fn imu_startup_without_subscriber_fails() {
        let mut communication_registry = CommunicationRegistry::new();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) =
            TrajectoryGenerator::run(1.0 / GENERATOR_FREQ, Arc::clone(&shutdown_trigger));
        let result = start_imu(
            Arc::clone(&generated_data_handle),
            &mut communication_registry,
            Arc::clone(&shutdown_trigger),
        );
        assert!(result.is_err());
    }

    #[test]
    fn imu_startup_with_subscriber_suceeds() {
        let (tx, _) = mpsc::channel();
        let mut communication_registry = CommunicationRegistry::new();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) =
            TrajectoryGenerator::run(1.0 / GENERATOR_FREQ, Arc::clone(&shutdown_trigger));

        communication_registry.register_for_input(DataSource::Imu, tx);
        let result = start_imu(
            Arc::clone(&generated_data_handle),
            &mut communication_registry,
            Arc::clone(&shutdown_trigger),
        );

        assert!(result.is_ok());
        shutdown_trigger.store(true, Ordering::SeqCst);
    }

    #[test]
    fn test_producer_sends_data() {
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) =
            TrajectoryGenerator::run(1.0 / GENERATOR_FREQ, Arc::clone(&shutdown_trigger));

        let mut communication_registry = CommunicationRegistry::new();
        let (tx, rx) = mpsc::channel();
        communication_registry.register_for_input(DataSource::Imu, tx);

        let test_producer_handle = start_imu(
            Arc::clone(&generated_data_handle),
            &mut communication_registry,
            Arc::clone(&shutdown_trigger),
        );

        thread::sleep(Duration::from_secs(2));

        system_shutdown(Arc::clone(&shutdown_trigger));
        test_producer_handle.unwrap().join().unwrap();

        let received: Vec<_> = rx.try_iter().collect();
        assert!(!received.is_empty());
    }
}
