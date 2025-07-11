use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use crate::{
    communication_registry::{CommunicationRegistry, DataSource},
    data::{Data, Telemetry},
    imu::Imu,
    trajectory_generator::TrajectoryGenerator,
    gps::Gps,
    kalman::{KalmanData, KalmanFilter},
};

mod communication_registry;
pub mod data;
mod gps;
mod imu;
mod trajectory_generator;
mod kalman;

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

fn start_gps(
    trajectory_data: Arc<Mutex<Data>>,
    communication_registry: &mut CommunicationRegistry,
    shutdown: Arc<AtomicBool>,
) -> Result<JoinHandle<()>, Error> {
    match communication_registry.get_registered_transmitters(DataSource::Gps) {
        Some(transmitters) => Ok(Gps::run(
            trajectory_data,
            transmitters,
            Arc::clone(&shutdown),
        )),
        None => Err(Error::StartupError(
            "No subscribers for GPS. Start aborted.",
        )),
    }
}

fn start_kalman(
    communication_registry: &mut CommunicationRegistry,
    shutdown: Arc<AtomicBool>,
) -> Result<JoinHandle<()>, Error> {
    let (tx_imu, input_rx) = mpsc::channel();
    let tx_gps = tx_imu.clone();
    communication_registry.register_for_input(DataSource::Imu, tx_imu);
    communication_registry.register_for_input(DataSource::Gps, tx_gps);

    match communication_registry.get_registered_transmitters(DataSource::Kalman) {
        Some(transmitters) => Ok(KalmanFilter::run(
            transmitters,
            input_rx,
            Arc::clone(&shutdown),
        )),
        None => Err(Error::StartupError(
            "No subscribers for Imu. Start aborted.",
        )),
    }
}

fn create_data_consumer(source: DataSource, consumer_registry: &mut CommunicationRegistry) -> JoinHandle<()> {
    let (tx, input_rx) = mpsc::channel();
    consumer_registry.register_for_input(source, tx);

    let handle = thread::spawn(move || {
        for data in input_rx {
            match data {
                Telemetry::Acceleration(d) => {
                    println!(
                        "Consumer{:?}: received: {}, {}, {}",
                        source,
                        d.x,
                        d.y,
                        d.z
                    )
                }
                Telemetry::Position(d) => {
                    println!(
                        "Consumer{:?}: received: {}, {}, {}",
                        source,
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

fn system_shutdown(
    map: &mut CommunicationRegistry,
    shutdown_trigger: Arc<AtomicBool>
) {
    shutdown_trigger.store(true, Ordering::SeqCst);
    for vec in map.transmitter_registry.values_mut() {
        vec.clear(); 
    }
}

fn main() -> Result<(), Error> {
    let mut communication_registry = CommunicationRegistry::new();
    let shutdown_trigger = Arc::new(AtomicBool::new(false));

    let consumer0_handle = create_data_consumer(DataSource::Gps, &mut communication_registry);
    let consumer1_handle = create_data_consumer(DataSource::Imu, &mut communication_registry);
    let consumer2_handle = create_data_consumer(DataSource::Kalman, &mut communication_registry);

    let kalman_handle = start_kalman(
        &mut communication_registry,
        Arc::clone(&shutdown_trigger),
    )?;
    let (generated_data_handle, generator_handle) =
        TrajectoryGenerator::run(1.0 / GENERATOR_FREQ, Arc::clone(&shutdown_trigger));
    let imu_handle = start_imu(
        Arc::clone(&generated_data_handle),
        &mut communication_registry,
        Arc::clone(&shutdown_trigger),
    )?;
    let gps_handle = start_gps(
        Arc::clone(&generated_data_handle),
        &mut communication_registry,
        Arc::clone(&shutdown_trigger),
    )?;

    thread::sleep(Duration::from_secs(6));
    system_shutdown(
        &mut communication_registry,
        Arc::clone(&shutdown_trigger)
    );

    generator_handle.join().unwrap();
    imu_handle.join().unwrap();
    gps_handle.join().unwrap();
    kalman_handle.join().unwrap();
    consumer2_handle.join().unwrap();
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
    fn gps_startup_without_subscriber_fails() {
        let mut communication_registry = CommunicationRegistry::new();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) =
            TrajectoryGenerator::run(1.0 / GENERATOR_FREQ, Arc::clone(&shutdown_trigger));
        let result = start_gps(
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
    fn gps_startup_with_subscriber_suceeds() {
        let (tx, _) = mpsc::channel();
        let mut communication_registry = CommunicationRegistry::new();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) =
            TrajectoryGenerator::run(1.0 / GENERATOR_FREQ, Arc::clone(&shutdown_trigger));

        communication_registry.register_for_input(DataSource::Gps, tx);
        let result = start_gps(
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
        system_shutdown(
            &mut communication_registry,
            Arc::clone(&shutdown_trigger)
        );
        test_producer_handle.unwrap().join().unwrap();

        let received: Vec<_> = rx.try_iter().collect();
        assert!(!received.is_empty());
    }
}
