use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use crate::{
    average::Average,
    communication_registry::{CommunicationRegistry, DataSource},
    data::{Data, Telemetry},
    gps::Gps,
    imu::Imu,
    kalman::KalmanFilter,
    logger::{get_data, log},
    trajectory_generator::TrajectoryGeneratorBuilder,
};

use chrono::{DateTime, Local};

mod average;
mod communication_registry;
pub mod data;
mod gps;
mod imu;
mod kalman;
mod logger;
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
) -> Result<JoinHandle<()>, Error> {
    let (tx_imu, input_rx) = mpsc::channel();
    let tx_gps = tx_imu.clone();
    communication_registry.register_for_input(DataSource::Imu, tx_imu);
    communication_registry.register_for_input(DataSource::Gps, tx_gps);

    match communication_registry.get_registered_transmitters(DataSource::Kalman) {
        Some(transmitters) => Ok(KalmanFilter::run(transmitters, input_rx)),
        None => Err(Error::StartupError(
            "No subscribers for Kalman. Start aborted.",
        )),
    }
}

fn start_avg_filter(
    communication_registry: &mut CommunicationRegistry,
) -> Result<JoinHandle<()>, Error> {
    let (tx, input_rx) = mpsc::channel();
    communication_registry.register_for_input(DataSource::Gps, tx);

    match communication_registry.get_registered_transmitters(DataSource::Average) {
        Some(transmitters) => Ok(Average::run(transmitters, input_rx)),
        None => Err(Error::StartupError(
            "No subscribers for Average filter. Start aborted.",
        )),
    }
}

fn create_data_consumer(
    source: DataSource,
    consumer_registry: &mut CommunicationRegistry,
) -> JoinHandle<()> {
    let (tx, input_rx) = mpsc::channel();
    consumer_registry.register_for_input(source, tx);

    log("Consumers", source);

    let handle = thread::spawn(move || {
        for data in input_rx {
            match data {
                Telemetry::Acceleration(d) => {
                    println!(
                        "Consuming from: {:?}: received: {}, {}, {}",
                        source, d.x, d.y, d.z
                    )
                }
                Telemetry::Position(d) => {
                    println!(
                        "Consuming from: {:?}: received: {}, {}, {}",
                        source, d.x, d.y, d.z
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

    let placeholder_consumer_handle =
        create_data_consumer(DataSource::Kalman, &mut communication_registry);
    let consumer3_handle = create_data_consumer(DataSource::Average, &mut communication_registry);

    let kalman_handle = start_kalman(&mut communication_registry)?;

    let avg_handle = start_avg_filter(&mut communication_registry)?;
    let (generated_data_handle, generator_handle) = TrajectoryGeneratorBuilder::new()
        .with_perlin_mode()
        .with_period(1.0 / GENERATOR_FREQ)
        .with_seed(2137)
        .spawn(Arc::clone(&shutdown_trigger));
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
    system_shutdown(Arc::clone(&shutdown_trigger));

    generator_handle.join().unwrap();
    imu_handle.join().unwrap();
    gps_handle.join().unwrap();
    kalman_handle.join().unwrap();
    avg_handle.join().unwrap();
    placeholder_consumer_handle.join().unwrap();
    consumer3_handle.join().unwrap();

    let consumers = get_data::<DataSource>("Consumers");

    if let Some(data) = consumers {
        for entry in data {
            let date_time: DateTime<Local> = entry.timestamp.into();
            println!(
                "Consumer created at: {}, who: {:?}",
                date_time.format("%Y-%m-%d %H:%M:%S"),
                entry.data
            );
        }
    } else {
        println!("No consumers created.");
    }

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
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_random_mode()
            .with_period(1.0 / GENERATOR_FREQ)
            .spawn(Arc::clone(&shutdown_trigger));
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
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_perlin_mode()
            .with_period(1.0 / GENERATOR_FREQ)
            .spawn(Arc::clone(&shutdown_trigger));
        let result = start_gps(
            Arc::clone(&generated_data_handle),
            &mut communication_registry,
            Arc::clone(&shutdown_trigger),
        );
        assert!(result.is_err());
    }

    #[test]
    fn kalman_startup_without_subscriber_fails() {
        let mut communication_registry = CommunicationRegistry::new();
        let result = start_kalman(&mut communication_registry);
        assert!(result.is_err());
    }

    #[test]
    fn avg_startup_without_subscriber_fails() {
        let mut communication_registry = CommunicationRegistry::new();
        let result = start_avg_filter(&mut communication_registry);
        assert!(result.is_err());
    }

    #[test]
    fn imu_startup_with_subscriber_suceeds() {
        let (tx, _) = mpsc::channel();
        let mut communication_registry = CommunicationRegistry::new();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_perlin_mode()
            .with_period(1.0 / GENERATOR_FREQ)
            .spawn(Arc::clone(&shutdown_trigger));

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
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_perlin_mode()
            .with_period(1.0 / GENERATOR_FREQ)
            .spawn(Arc::clone(&shutdown_trigger));

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
    fn avg_startup_with_subscriber_suceeds() {
        let (tx, _) = mpsc::channel();
        let mut communication_registry = CommunicationRegistry::new();

        communication_registry.register_for_input(DataSource::Average, tx);
        let result = start_avg_filter(&mut communication_registry);

        assert!(result.is_ok());
    }

    #[test]
    fn kalman_startup_with_subscriber_suceeds() {
        let (tx, _) = mpsc::channel();
        let mut communication_registry = CommunicationRegistry::new();

        communication_registry.register_for_input(DataSource::Kalman, tx);
        let result = start_kalman(&mut communication_registry);

        assert!(result.is_ok());
    }

    #[test]
    fn test_producer_sends_data() {
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_perlin_mode()
            .with_period(1.0 / GENERATOR_FREQ)
            .spawn(Arc::clone(&shutdown_trigger));

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
