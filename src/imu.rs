use crate::{
    data::{Data, Telemetry},
    logger::log,
    utils::get_cycle_duration,
};
use nalgebra::Vector3;
use std::{
    num::NonZeroU32,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::Sender,
        Arc, Mutex,
    },
    thread::JoinHandle,
    time::{Duration, SystemTimeError},
};

use rand::rng;
use rand_distr::{Distribution, Normal};

const LOGGER_PREFIX: &str = "IMU";

pub struct Imu {
    tx: Vec<Sender<Telemetry>>,
    position_data: Arc<Mutex<Data>>,
    prev_position: Data,
    prev_velocity: Vector3<f64>,
    last_valid_acceleration: Vector3<f64>,
    frequency: NonZeroU32,
    noise_generator: Normal<f64>,
}

impl Imu {
    pub fn run(
        position_data: Arc<Mutex<Data>>,
        tx: Vec<Sender<Telemetry>>,
        shutdown: Arc<AtomicBool>,
        frequency: NonZeroU32,
        noise_standard_deviation: f64,
    ) -> JoinHandle<()> {
        let mut imu = Imu::new(position_data, tx, frequency, noise_standard_deviation);
        std::thread::spawn(move || match imu.init_velocity() {
            Ok(_) => {
                // wait one cycle before entering the main loop to give the trajectory generator
                // a chance to update position after calculating initial velocity
                std::thread::sleep(get_cycle_duration(imu.frequency));
                while should_run(&shutdown, &imu.tx) {
                    if let Err(e) = imu.step() {
                        eprintln!("Imu internal error: {e}. Aborting.");
                        return;
                    };
                    std::thread::sleep(get_cycle_duration(imu.frequency));
                }
                println!("Imu removed");
            }
            Err(e) => {
                eprintln!("Imu internal error during initialization: {e}. Aborting.");
            }
        })
    }

    fn new(
        position_data: Arc<Mutex<Data>>,
        tx: Vec<Sender<Telemetry>>,
        frequency: NonZeroU32,
        noise_standard_deviation: f64,
    ) -> Imu {
        Imu {
            tx,
            position_data: Arc::clone(&position_data),
            prev_position: *position_data.lock().unwrap(),
            prev_velocity: Vector3::new(0.0, 0.0, 0.0),
            last_valid_acceleration: Vector3::new(0.0, 0.0, 0.0),
            frequency,
            noise_generator: Normal::new(0.0, noise_standard_deviation).unwrap(),
        }
    }

    fn init_velocity(&mut self) -> Result<(), SystemTimeError> {
        // sleep one cycle to give trajectory generator a chance to update position
        std::thread::sleep(get_cycle_duration(self.frequency));
        let current_position = *self.position_data.lock().unwrap();

        let delta_time = current_position
            .timestamp
            .duration_since(self.prev_position.timestamp)?;

        if !delta_time.is_zero() {
            self.prev_velocity =
                calculate_velocity(&self.prev_position, &current_position, &delta_time);
        }
        Ok(())
    }

    fn step(&mut self) -> Result<(), SystemTimeError> {
        let current_position = *self.position_data.lock().unwrap();

        let delta_time = current_position
            .timestamp
            .duration_since(self.prev_position.timestamp)?;

        if !delta_time.is_zero() {
            let current_velocity =
                calculate_velocity(&self.prev_position, &current_position, &delta_time);
            self.last_valid_acceleration =
                calculate_acceleration(&self.prev_velocity, &current_velocity, &delta_time);
            self.prev_velocity = current_velocity;
        }

        self.prev_position = current_position;

        let data_to_send = Data {
            x: self.last_valid_acceleration.x + self.noise_generator.sample(&mut rng()),
            y: self.last_valid_acceleration.y + self.noise_generator.sample(&mut rng()),
            z: self.last_valid_acceleration.z + self.noise_generator.sample(&mut rng()),
            timestamp: current_position.timestamp,
        };

        log(LOGGER_PREFIX, data_to_send);
        self.send_data(data_to_send);
        Ok(())
    }

    fn send_data(&mut self, data: Data) {
        self.tx
            .retain(|tx| tx.send(Telemetry::Acceleration(data)).is_ok());
    }
}

fn should_run(shutdown_flag: &Arc<AtomicBool>, transmitters: &[Sender<Telemetry>]) -> bool {
    !shutdown_flag.load(Ordering::SeqCst) && !transmitters.is_empty()
}

fn calculate_velocity(
    prev_position: &Data,
    current_position: &Data,
    delta_time: &Duration,
) -> Vector3<f64> {
    let x = (current_position.x - prev_position.x) / delta_time.as_secs_f64();
    let y = (current_position.y - prev_position.y) / delta_time.as_secs_f64();
    let z = (current_position.z - prev_position.z) / delta_time.as_secs_f64();
    Vector3::new(x, y, z)
}

fn calculate_acceleration(
    prev_velocity: &Vector3<f64>,
    current_velocity: &Vector3<f64>,
    delta_time: &Duration,
) -> Vector3<f64> {
    (current_velocity - prev_velocity) / delta_time.as_secs_f64()
}

#[cfg(test)]
mod test {
    use ntest_timeout::timeout;
    use std::{sync::mpsc, time::SystemTime};

    use super::*;

    #[test]
    fn given_zero_initial_velocity_expect_correct_acceleration_calculation() {
        let initial_velocity = Vector3::new(0.0, 0.0, 0.0);
        let final_velocity = Vector3::new(1.0, 2.0, 3.0);
        let delta_time = Duration::from_secs(1);
        let expected_acceleration = Vector3::new(1.0, 2.0, 3.0);

        let results = calculate_acceleration(&initial_velocity, &final_velocity, &delta_time);
        for (result, expected) in results.iter().zip(expected_acceleration.iter()) {
            approx::assert_abs_diff_eq!(result, expected);
        }
    }

    #[test]
    fn given_nonzero_initial_velocity_expect_correct_acceleration_calculation() {
        let initial_velocity = Vector3::new(0.5, 3.0, 1.0);
        let final_velocity = Vector3::new(1.0, 2.0, 3.0);
        let delta_time = Duration::from_secs(2);
        let expected_acceleration = Vector3::new(0.25, -0.5, 1.0);

        let results = calculate_acceleration(&initial_velocity, &final_velocity, &delta_time);
        for (result, expected) in results.iter().zip(expected_acceleration.iter()) {
            approx::assert_abs_diff_eq!(result, expected);
        }
    }

    #[test]
    fn given_two_positions_expect_valid_velocity_calculated() {
        let initial_position = Data {
            x: 1.0,
            y: 2.0,
            z: 5.0,
            timestamp: SystemTime::now(),
        };
        let final_position = Data {
            x: 2.0,
            y: 0.0,
            z: 5.0,
            timestamp: SystemTime::now(),
        };
        let delta_time = Duration::from_secs_f64(2.0);
        let expected_velocity = Vector3::new(0.5, -1.0, 0.0);

        let result = calculate_velocity(&initial_position, &final_position, &delta_time);
        for (result, expected) in result.iter().zip(expected_velocity.iter()) {
            approx::assert_abs_diff_eq!(result, expected);
        }
    }

    #[test]
    fn given_velocity_initialization_expect_acceleration_to_be_unchanged() {
        let position_data = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let arbitrary_frequency = NonZeroU32::new(1).unwrap();
        let mut imu = Imu {
            tx: vec![tx],
            position_data: Arc::clone(&position_data),
            prev_position: *position_data.lock().unwrap(),
            prev_velocity: Vector3::new(0.0, 0.0, 0.0),
            last_valid_acceleration: Vector3::new(0.0, 0.0, 0.0),
            frequency: arbitrary_frequency,
            noise_generator: Normal::new(0.0, 0.0).unwrap(),
        };

        {
            let mut position_data = position_data.lock().unwrap();
            position_data.x = 1.0;
            position_data.y = 2.0;
            position_data.z = 3.0;
            position_data.timestamp += Duration::from_secs(1);
        }

        assert!(imu.init_velocity().is_ok());

        approx::assert_abs_diff_eq!(imu.prev_velocity.x, 1.0);
        approx::assert_abs_diff_eq!(imu.prev_velocity.y, 2.0);
        approx::assert_abs_diff_eq!(imu.prev_velocity.z, 3.0);
        approx::assert_abs_diff_eq!(imu.last_valid_acceleration.x, 0.0);
        approx::assert_abs_diff_eq!(imu.last_valid_acceleration.y, 0.0);
        approx::assert_abs_diff_eq!(imu.last_valid_acceleration.z, 0.0);
        let received_elements = rx.try_iter().collect::<Vec<_>>();
        assert!(received_elements.is_empty());
    }

    #[test]
    fn given_rx_goes_out_of_scope_imu_shuts_down() {
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let position_data = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let arbitrary_frequency = NonZeroU32::new(1).unwrap();
        let imu = Imu::run(
            Arc::clone(&position_data),
            vec![tx],
            Arc::clone(&shutdown_trigger),
            arbitrary_frequency,
            0.0,
        );
        drop(rx);
        imu.join().unwrap();
    }

    #[test]
    fn given_no_shutdown_signal_and_transmitters_presend_expect_run() {
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let (tx, _) = mpsc::channel();
        assert!(should_run(&shutdown_trigger, &[tx]));
    }

    #[test]
    fn given_shutdown_signal_expect_stop() {
        let shutdown_trigger = Arc::new(AtomicBool::new(true));
        let (tx, _) = mpsc::channel();
        assert!(!should_run(&shutdown_trigger, &[tx]));
    }

    #[test]
    fn given_no_transmitters_signal_expect_stop() {
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let empty: Vec<Sender<Telemetry>> = Vec::new();
        assert!(!should_run(&shutdown_trigger, &empty));
    }

    #[test]
    fn test_step() {
        let position_data = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let arbitrary_frequency = NonZeroU32::new(1).unwrap();
        let no_noise = Normal::new(0.0, 0.0).unwrap();
        let mut imu = Imu {
            tx: vec![tx],
            position_data: Arc::clone(&position_data),
            prev_position: *position_data.lock().unwrap(),
            prev_velocity: Vector3::new(0.0, 0.0, 0.0),
            last_valid_acceleration: Vector3::new(0.0, 0.0, 0.0),
            frequency: arbitrary_frequency,
            noise_generator: no_noise,
        };

        {
            let mut position_data = position_data.lock().unwrap();
            position_data.x = 1.0;
            position_data.y = 2.0;
            position_data.z = 3.0;
            position_data.timestamp += Duration::from_secs(1);
        }
        assert!(imu.step().is_ok());

        let Telemetry::Acceleration(acc) = rx.recv().unwrap() else {
            panic!("Cannot return position!");
        };
        approx::assert_abs_diff_eq!(acc.x, 1.0);
        approx::assert_abs_diff_eq!(acc.y, 2.0);
        approx::assert_abs_diff_eq!(acc.z, 3.0);

        {
            let mut position_data = position_data.lock().unwrap();
            position_data.x = 1.5;
            position_data.y = 1.5;
            position_data.z = 2.0;
            position_data.timestamp += Duration::from_secs(1);
        }

        assert!(imu.step().is_ok());
        let Telemetry::Acceleration(acc) = rx.recv().unwrap() else {
            panic!("Cannot return position!");
        };
        approx::assert_abs_diff_eq!(acc.x, -0.5);
        approx::assert_abs_diff_eq!(acc.y, -2.5);
        approx::assert_abs_diff_eq!(acc.z, -4.0);
    }

    #[test]
    fn given_next_timestamp_is_behind_previous_expect_run_to_return_err() {
        let initial_velocity = Vector3::new(0.0, 0.0, 0.0);
        let position_data = Arc::new(Mutex::new(Data::new()));
        let arbitrary_frequency = NonZeroU32::new(1).unwrap();
        let (tx, _) = mpsc::channel();
        let mut imu = Imu {
            tx: vec![tx],
            position_data: Arc::clone(&position_data),
            prev_position: *position_data.lock().unwrap(),
            prev_velocity: initial_velocity,
            last_valid_acceleration: Vector3::new(0.0, 0.0, 0.0),
            frequency: arbitrary_frequency,
            noise_generator: Normal::new(0.0, 0.0).unwrap(),
        };
        position_data.lock().unwrap().timestamp -= Duration::new(1, 0);

        assert!(imu.step().is_err());
    }

    #[test]
    fn given_the_same_timestamp_expect_acceleration_from_last_valid_measurement() {
        let position_data = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let arbitrary_frequency = NonZeroU32::new(1).unwrap();
        let mut imu = Imu {
            tx: vec![tx],
            position_data: Arc::clone(&position_data),
            prev_position: *position_data.lock().unwrap(),
            prev_velocity: Vector3::new(0.0, 0.0, 0.0),
            last_valid_acceleration: Vector3::new(0.0, 0.0, 0.0),
            frequency: arbitrary_frequency,
            noise_generator: Normal::new(0.0, 0.0).unwrap(),
        };

        {
            let mut position_data = position_data.lock().unwrap();
            position_data.x = 1.0;
            position_data.y = 2.0;
            position_data.z = 3.0;
            position_data.timestamp += Duration::from_secs(1);
        }
        assert!(imu.step().is_ok());

        let Telemetry::Acceleration(acc) = rx.recv().unwrap() else {
            panic!("Cannot return position!");
        };
        approx::assert_abs_diff_eq!(acc.x, 1.0);
        approx::assert_abs_diff_eq!(acc.y, 2.0);
        approx::assert_abs_diff_eq!(acc.z, 3.0);

        {
            let mut position_data = position_data.lock().unwrap();
            position_data.x = 1.5;
            position_data.y = 1.5;
            position_data.z = 2.0;
        }

        assert!(imu.step().is_ok());
        let Telemetry::Acceleration(acc) = rx.recv().unwrap() else {
            panic!("Cannot return position!");
        };
        approx::assert_abs_diff_eq!(acc.x, 1.0);
        approx::assert_abs_diff_eq!(acc.y, 2.0);
        approx::assert_abs_diff_eq!(acc.z, 3.0);
    }

    #[test]
    #[timeout(10000)]
    fn given_next_timestamp_is_behind_previous_expect_imu_to_turn_off() {
        let position_data = Arc::new(Mutex::new(Data::new()));
        let shutdown = Arc::new(AtomicBool::new(false));
        let (tx, _) = mpsc::channel();
        let two_hertz_frequency = NonZeroU32::new(2).unwrap();
        let imu = Imu::run(
            Arc::clone(&position_data),
            vec![tx],
            Arc::clone(&shutdown),
            two_hertz_frequency,
            0.0,
        );

        position_data.lock().unwrap().timestamp -= Duration::new(1, 0);

        imu.join().unwrap();
    }

    #[test]
    fn given_noise_enabled_expect_output_with_noise() {
        let position_data = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let arbitrary_frequency = NonZeroU32::new(1).unwrap();
        let noise_generator = Normal::new(0.0, 5.0).unwrap();
        let mut imu = Imu {
            tx: vec![tx],
            position_data: Arc::clone(&position_data),
            prev_position: *position_data.lock().unwrap(),
            prev_velocity: Vector3::new(0.0, 0.0, 0.0),
            last_valid_acceleration: Vector3::new(0.0, 0.0, 0.0),
            frequency: arbitrary_frequency,
            noise_generator,
        };

        {
            let mut position_data = position_data.lock().unwrap();
            position_data.x = 1.0;
            position_data.y = 2.0;
            position_data.z = 3.0;
            position_data.timestamp += Duration::from_secs(1);
        }
        assert!(imu.step().is_ok());

        let Telemetry::Acceleration(acc) = rx.recv().unwrap() else {
            panic!("Cannot return position!");
        };
        approx::assert_abs_diff_ne!(acc.x, 1.0);
        approx::assert_abs_diff_ne!(acc.y, 2.0);
        approx::assert_abs_diff_ne!(acc.z, 3.0);
    }
}
