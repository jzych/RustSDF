use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex,
    },
    thread::JoinHandle,
    time::SystemTime,
};

use crate::{
    communication_registry::{CommunicationRegistry, DataSource},
    config::*,
    data::{Data, Telemetry},
    estimator_builder::EstimatorBuilder,
    logger::log,
    log_config::*,
    sensor_builder::SensorBuilder,
    trajectory_generator::TrajectoryGeneratorBuilder,
    csv_handler::*,
    visualization::{PlotterReceivers, real_time_visualization::RealTimeVisualization, static_visualization::StaticVisualization},
};

use estimators::kalman;
use estimators::inertial_navigator;

mod average;
mod communication_registry;
mod config;
mod csv_handler;
pub mod data;
mod estimator_builder;
mod gps;
mod imu;
mod log_config;
mod logger;
mod sensor_builder;
mod trajectory_generator;
mod utils;
mod visualization;
mod periodic_runner;
mod estimators;

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
    let Some(subscribers) = communication_registry.get_registered_transmitters(DataSource::Imu)
    else {
        return Err(Error::StartupError(
            "No subscribers for IMU. Start aborted.",
        ));
    };
    Ok(SensorBuilder::new_imu()
        .with_frequency(IMU_FREQ)
        .with_position_generator(trajectory_data)
        .with_subscribers(subscribers)
        .with_output_noise(IMU_OUTPUT_NOISE_SIGMA)
        .spawn(shutdown))
}

fn start_gps(
    trajectory_data: Arc<Mutex<Data>>,
    communication_registry: &mut CommunicationRegistry,
    shutdown: Arc<AtomicBool>,
) -> Result<JoinHandle<()>, Error> {
    let Some(subscribers) = communication_registry.get_registered_transmitters(DataSource::Gps)
    else {
        return Err(Error::StartupError(
            "No subscribers for GPS. Start aborted.",
        ));
    };
    Ok(SensorBuilder::new_gps()
        .with_frequency(GPS_FREQ)
        .with_position_generator(trajectory_data)
        .with_subscribers(subscribers)
        .with_output_noise(GPS_OUTPUT_NOISE_SIGMA)
        .spawn(shutdown))
}

fn start_kalman(
    communication_registry: &mut CommunicationRegistry,
) -> Result<JoinHandle<()>, Error> {
    let (tx_imu, input_rx) = mpsc::channel();
    let tx_gps = tx_imu.clone();
    communication_registry.register_for_input(DataSource::Imu, tx_imu);
    communication_registry.register_for_input(DataSource::Gps, tx_gps);

    match communication_registry.get_registered_transmitters(DataSource::Kalman) {
        Some(subscribers) => Ok(EstimatorBuilder::new_kalman()
            .with_subscribers(subscribers)
            .with_input_rx(input_rx)
            .spawn()),
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
        Some(subscribers) => Ok(EstimatorBuilder::new_average(BUFFER_LENGTH)
            .with_subscribers(subscribers)
            .with_input_rx(input_rx)
            .spawn()),
        None => Err(Error::StartupError(
            "No subscribers for Average filter. Start aborted.",
        )),
    }
}

fn start_inertial_navigator(
    communication_registry: &mut CommunicationRegistry,
) -> Result<JoinHandle<()>, Error> {
    let (tx_imu, input_rx) = mpsc::channel();
    let tx_gps = tx_imu.clone();
    communication_registry.register_for_input(DataSource::Imu, tx_imu);
    communication_registry.register_for_input(DataSource::Gps, tx_gps);

    match communication_registry.get_registered_transmitters(DataSource::InertialNavigator) {
        Some(subscribers) => Ok(EstimatorBuilder::new_inertial_navigator()
            .with_subscribers(subscribers)
            .with_input_rx(input_rx)
            .spawn()),
        None => Err(Error::StartupError(
            "No subscribers for Average filter. Start aborted.",
        )),
    }
}

fn start_static_visualization(
    communication_registry: &mut CommunicationRegistry,
    simulation_start: SystemTime,
) -> JoinHandle<()> {
    let (tx_avg, rx_avg) = mpsc::channel();
    let (tx_kalman, rx_kalman) = mpsc::channel();
    let (tx_gps, rx_gps) = mpsc::channel();
    let (tx_inertial, rx_inertial) = mpsc::channel();
    let (tx_groundtruth, rx_groundtruth) = mpsc::channel();
    communication_registry.register_for_input(DataSource::Average, tx_avg);
    communication_registry.register_for_input(DataSource::Kalman, tx_kalman);
    communication_registry.register_for_input(DataSource::Gps, tx_gps);
    communication_registry.register_for_input(DataSource::InertialNavigator, tx_inertial);
    communication_registry.register_for_input(DataSource::Groundtruth, tx_groundtruth);

    StaticVisualization::run(PlotterReceivers { rx_gps, rx_avg, rx_kalman, rx_inertial, rx_groundtruth}, simulation_start)
}

fn start_trajectory_generator(
    consumer_registry: &mut CommunicationRegistry,
    shutdown_trigger: Arc<AtomicBool>,
) -> (Arc<Mutex<Data>>, JoinHandle<()>) {
    let subscribers = consumer_registry
        .get_registered_transmitters(DataSource::Groundtruth)
        .unwrap_or_default();
    TrajectoryGeneratorBuilder::new()
        .with_frequency(GENERATOR_FREQ)
        .with_perlin_mode()
        .with_subscribers(subscribers)
        .spawn(Arc::clone(&shutdown_trigger))
}

fn system_shutdown(shutdown_trigger: Arc<AtomicBool>) {
    shutdown_trigger.store(true, Ordering::SeqCst);
}

fn register_dynamic_plot(
    communication_registry: &mut CommunicationRegistry,
) -> (PlotterReceivers, SystemTime) {
    let simulation_start = SystemTime::now();
    let (tx_gps, rx_gps) = mpsc::channel();
    let (tx_avg, rx_avg) = mpsc::channel();
    let (tx_kalman, rx_kalman) = mpsc::channel();
    let (tx_inertial, rx_inertial) = mpsc::channel();
    let (tx_groundtruth, rx_groundtruth) = mpsc::channel();

    communication_registry.register_for_input(DataSource::Gps, tx_gps);
    communication_registry.register_for_input(DataSource::Average, tx_avg);
    communication_registry.register_for_input(DataSource::Kalman, tx_kalman);
    communication_registry.register_for_input(DataSource::InertialNavigator, tx_inertial);
    communication_registry.register_for_input(DataSource::Groundtruth, tx_groundtruth);

    (
        PlotterReceivers {
            rx_gps,
            rx_avg,
            rx_kalman,
            rx_inertial,
            rx_groundtruth,
        },
        simulation_start,
    )
}

fn main() -> Result<(), Error> {
    log(GENERAL_LOG, "System start".to_string());
    let mut communication_registry = CommunicationRegistry::new();
    let shutdown_trigger = Arc::new(AtomicBool::new(false));
    let (receivers, simulation_start) = register_dynamic_plot(&mut communication_registry);
    let static_visu_handle = start_static_visualization(&mut communication_registry, simulation_start);

    let (generated_data_handle, generator_handle) =
        start_trajectory_generator(&mut communication_registry, Arc::clone(&shutdown_trigger));

    let kalman_handle = start_kalman(&mut communication_registry)?;
    let avg_handle = start_avg_filter(&mut communication_registry)?;
    let inertial_navigator_handle = start_inertial_navigator(&mut communication_registry)?;

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

    RealTimeVisualization::run(receivers, simulation_start);
    system_shutdown(Arc::clone(&shutdown_trigger));

    generator_handle.join().unwrap();
    imu_handle.join().unwrap();
    gps_handle.join().unwrap();
    kalman_handle.join().unwrap();
    avg_handle.join().unwrap();
    inertial_navigator_handle.join().unwrap();
    static_visu_handle.join().unwrap();

    save_logs_to_file();

    Ok(())
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc;
    use std::time::Duration;
    use std::thread;

    use super::*;

    #[test]
    fn imu_startup_without_subscriber_fails() {
        let mut communication_registry = CommunicationRegistry::new();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_random_mode()
            .with_frequency(GENERATOR_FREQ)
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
            .with_frequency(GENERATOR_FREQ)
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
    fn intertial_nav_startup_without_subscriber_fails() {
        let mut communication_registry = CommunicationRegistry::new();
        let result = start_inertial_navigator(&mut communication_registry);
        assert!(result.is_err());
    }

    #[test]
    fn imu_startup_with_subscriber_suceeds() {
        let (tx, _) = mpsc::channel();
        let mut communication_registry = CommunicationRegistry::new();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_perlin_mode()
            .with_frequency(GENERATOR_FREQ)
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
            .with_frequency(GENERATOR_FREQ)
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
    fn inertial_nav_startup_with_subscriber_suceeds() {
        let (tx, _) = mpsc::channel();
        let mut communication_registry = CommunicationRegistry::new();

        communication_registry.register_for_input(DataSource::InertialNavigator, tx);
        let result = start_inertial_navigator(&mut communication_registry);

        assert!(result.is_ok());
    }

    #[test]
    fn test_producer_sends_data() {
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (generated_data_handle, _) = TrajectoryGeneratorBuilder::new()
            .with_perlin_mode()
            .with_frequency(GENERATOR_FREQ)
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
