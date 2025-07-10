use crate::data::{Data, Telemetry};
use crate::trajectory_generator::TrajectoryGenerator;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

pub struct Imu {
    tx: Vec<Sender<Telemetry>>,
    trajectory_generator: Arc<Mutex<TrajectoryGenerator>>,
    prev_position: Data,
}

//Refresh rate in Hz
const REFRESH_FREQ: u32 = 2;

impl Imu {
    fn new(
        trajectory_generator: Arc<Mutex<TrajectoryGenerator>>,
        tx: Vec<Sender<Telemetry>>,
    ) -> Imu {
        Imu {
            tx,
            trajectory_generator,
            prev_position: Data::new(),
        }
    }

    pub fn run(
        trajectory_generator: Arc<Mutex<TrajectoryGenerator>>,
        tx: Vec<Sender<Telemetry>>,
        shutdown: Arc<AtomicBool>,
    ) -> JoinHandle<()> {
        std::thread::spawn(move || {
            let mut imu = Imu::new(trajectory_generator, tx);
            while !shutdown.load(Ordering::SeqCst) {
                let current_position = imu
                    .trajectory_generator
                    .lock()
                    .unwrap()
                    .get_current_position();

                let acceleration = calculate_acceleration(&imu.prev_position, &current_position);
                imu.prev_position = current_position;

                imu.tx.retain(|tx| tx.send(acceleration).is_ok());
                if imu.tx.is_empty() {
                    break;
                }
                std::thread::sleep(get_cycle_duration(REFRESH_FREQ));
            }
        })
    }
}

fn calculate_acceleration(previous_position: &Data, current_position: &Data) -> Telemetry {
    let delta_time = current_position
        .timestamp
        .duration_since(previous_position.timestamp)
        .expect("Clock may have gone backwards");

    let a_x = calculate_axis_acceleration(previous_position.x, current_position.x, &delta_time);
    let a_y = calculate_axis_acceleration(previous_position.y, current_position.y, &delta_time);
    let a_z = calculate_axis_acceleration(previous_position.z, current_position.z, &delta_time);
    Telemetry::Acceleration(Data {
        x: a_x,
        y: a_y,
        z: a_z,
        timestamp: current_position.timestamp,
    })
}

///
/// acceleration calculated from s=(at^2)/2 equation
///
fn calculate_axis_acceleration(
    prev_position: f64,
    curr_position: f64,
    delta_time: &Duration,
) -> f64 {
    (2.0 * (curr_position - prev_position)) / delta_time.as_secs_f64().powf(2.0)
}

fn get_cycle_duration(frequency: u32) -> Duration {
    if frequency == 0 {
        panic!("Frequency cannot be set to zero");
    }
    Duration::from_secs_f64(1.0 / frequency as f64)
}

#[cfg(test)]
mod test {
    use std::{sync::mpsc, time::SystemTime};

    use super::*;

    #[test]
    fn axis_acceleration_calculation() {
        let start_position = 0.0;
        let finish_position = 1.0;
        let time_elapsed = Duration::from_secs_f64(1.0);
        let expected_acceleration = 2.0;

        approx::assert_abs_diff_eq!(
            calculate_axis_acceleration(start_position, finish_position, &time_elapsed),
            expected_acceleration
        );
    }

    #[test]
    fn acceleration_calculation_for_3d() {
        let current_time = SystemTime::now();
        let start = Data {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            timestamp: current_time,
        };
        let finish = Data {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            timestamp: current_time + Duration::from_secs(1),
        };

        let Telemetry::Acceleration(result) = calculate_acceleration(&start, &finish) else {
            panic!("Expected acceleration");
        };
        approx::assert_abs_diff_eq!(result.x, 2.0);
        approx::assert_abs_diff_eq!(result.y, 4.0);
        approx::assert_abs_diff_eq!(result.z, 6.0);
        assert_eq!(finish.timestamp, result.timestamp);
    }

    #[test]
    fn given_frequency_expect_cycle_duration() {
        let frequency = 10;
        let expected_cycle_duration = Duration::from_millis(100);
        assert_eq!(get_cycle_duration(frequency), expected_cycle_duration);
    }

    #[test]
    #[should_panic]
    fn given_zero_frequency_expect_panic() {
        let frequency = 0;
        get_cycle_duration(frequency);
    }

    #[test]
    fn given_rx_goes_out_of_scope_imu_shuts_down() {
        let trajectory_generator = Arc::new(Mutex::new(TrajectoryGenerator));
        let (tx, rx) = mpsc::channel();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let imu = Imu::run(
            trajectory_generator,
            vec![tx],
            Arc::clone(&shutdown_trigger),
        );
        drop(rx);
        imu.join().unwrap();
    }

    #[test]
    fn smoke_test_if_main_loop_does_not_crash() {
        let trajectory_generator = Arc::new(Mutex::new(TrajectoryGenerator));
        let (tx, rx) = mpsc::channel();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let imu = Imu::run(
            trajectory_generator,
            vec![tx],
            Arc::clone(&shutdown_trigger),
        );
        std::thread::sleep(Duration::from_secs(1));
        shutdown_trigger.store(true, Ordering::SeqCst);
        imu.join().unwrap();
        let received: Vec<_> = rx.try_iter().collect();
        assert!(!received.is_empty());
    }
}
