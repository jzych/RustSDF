#![allow(non_snake_case)]

use std::{
    sync::mpsc::{Receiver, Sender},
    thread::JoinHandle,
    time::{Duration, SystemTime},
};
use nalgebra::{Const, Matrix3, Matrix3x6, Matrix3x1, Matrix6, Matrix6x1, Matrix6x3};
use crate::{
    config::{IMU_FREQ, KALMAN_ACC_SIGMA, KALMAN_GPS_SIGMA, KALMAN_TIMING_TOLERANCE},
    data::{Data, Telemetry},
    logger::log,
    log_config::{GENERAL_LOG, KALMAN_LOG},
    utils::*,
};
use super::initialize_state_using_gps_data;

#[derive(Debug, Copy, Clone)]
pub struct KalmanData {
    x: Matrix6x1<f64>,
    P: Matrix6<f64>,
}

impl KalmanData {
    pub fn new() -> Self{
        Self {
            x: Matrix6x1::zeros_generic(Const::<6>, Const::<1>),
            P: create_matrix_Q(
                get_cycle_duration_f64(IMU_FREQ),
                KALMAN_ACC_SIGMA
            ) * 10000.0, // a big number to start with arbitrarily uncertain state estimation        
        }
    }
}

pub struct KalmanFilter {
    tx: Vec<Sender<Telemetry>>,
    A: Matrix6<f64>,
    B: Matrix6x3<f64>,
    H: Matrix3x6<f64>,
    Q: Matrix6<f64>,
    R: Matrix3<f64>,
    state: KalmanData,
}

impl KalmanFilter {

    fn new(tx: Vec<Sender<Telemetry>>) -> KalmanFilter {
        
        KalmanFilter {
            tx,
            A: create_matrix_A(get_cycle_duration_f64(IMU_FREQ)),
            B: create_matrix_B(get_cycle_duration_f64(IMU_FREQ)),
            H: create_matrix_H(),
            Q: create_matrix_Q(get_cycle_duration_f64(IMU_FREQ), KALMAN_ACC_SIGMA),
            R: create_matrix_R(KALMAN_GPS_SIGMA),
            state: KalmanData::new(),
        }
    }

    #[allow(dead_code)]
    pub fn show(&self){
        println!("A: {}", self.A);
        println!("B: {}", self.B);
        println!("H: {}", self.H);
        println!("Q: {}", self.Q);
        println!("R: {}", self.R);
    }

    pub fn run(
        tx: Vec<Sender<Telemetry>>,
        rx: Receiver<Telemetry>,
    ) -> JoinHandle<()> {
        let mut kalman = KalmanFilter::new(tx);
        let mut last_imu_data_timestamp = SystemTime::now();
        let mut gps_samples_received : u32 = 0;
        let mut prev_gps_data : Data = Data::new();

        std::thread::spawn( move || {
            for telemetry in &rx {
                initialize_state_using_gps_data(
                        telemetry,
                        &mut gps_samples_received,
                        &mut kalman.state.x,
                        &mut prev_gps_data,
                );
                if gps_samples_received == 2 {
                    break;
                }
            }
            for telemetry in rx {
                if telemetry_check(
                    telemetry,
                    &mut last_imu_data_timestamp,
                ) {
                    match telemetry {                    
                        Telemetry::Acceleration(data) => {
                            // prediction
                            // println!(
                            //     "Kalman received IMU data: {}, {}, {}",
                            //     data.x, data.y, data.z
                            // );
                            let u = Matrix3x1::new(data.x, data.y, data.z);
                            kalman.state.x = kalman.A * kalman.state.x + kalman.B * u;
                            kalman.state.P = kalman.A * kalman.state.P * kalman.A.transpose() + kalman.Q;
                        }
                        Telemetry::Position(data) => {
                            // correction
                            // println!(
                            //     "Kalman received GPS data: {}, {}, {}",
                            //     data.x, data.y, data.z
                            // );
                            let z = Matrix3x1::new(data.x, data.y, data.z);
                            let K = kalman.state.P * kalman.H.transpose() * (kalman.H * kalman.state.P * kalman.H.transpose() + kalman.R).try_inverse().unwrap();
                            kalman.state.x = kalman.state.x + K * (z - kalman.H * kalman.state.x);
                            kalman.state.P = (Matrix6::identity_generic(Const::<6>,Const::<6>) - K * kalman.H) * kalman.state.P;
                        },
                    }
                    
                    let kalman_position_estimate = Telemetry::Position(Data {
                        x: kalman.state.x[0],
                        y: kalman.state.x[1],
                        z: kalman.state.x[2],
                        timestamp: SystemTime::now()
                    });

                    kalman.tx.retain(|tx| tx.send(kalman_position_estimate).is_ok());
                    if kalman.tx.is_empty() {
                        break;
                    }
                    log(KALMAN_LOG, kalman_position_estimate);
                }
            }
            log(GENERAL_LOG, "Kalman filter removed".to_string());
        })
    }   
}

fn max_expected_imu_interval() -> Duration {
    Duration::from_secs_f64(get_cycle_duration_f64(IMU_FREQ) * (1.0 + KALMAN_TIMING_TOLERANCE))
}

fn min_expected_imu_interval() -> Duration {
    Duration::from_secs_f64(get_cycle_duration_f64(IMU_FREQ) * (1.0 - KALMAN_TIMING_TOLERANCE))
}

fn telemetry_check(
    telemetry: Telemetry,
    last_imu_data_timestamp: &mut SystemTime,
) -> bool {
    match telemetry {                    
        Telemetry::Acceleration(_) => {
            let current_imu_data_timestamp = SystemTime::now();
            let imu_elapsed: Duration = current_imu_data_timestamp.duration_since(*last_imu_data_timestamp).unwrap(); 
            *last_imu_data_timestamp = current_imu_data_timestamp;

            if imu_elapsed > max_expected_imu_interval() {
                eprintln!("Kalman: IMU data is late! Previous data obtained {}s {:03}ms ago. ",
                    imu_elapsed.as_secs(),
                    imu_elapsed.subsec_millis()
                );
                true
            } else if imu_elapsed >= min_expected_imu_interval() {
                true
            } else if imu_elapsed > Duration::from_secs_f64(0.0) {
                eprintln!("Kalman: IMU data received too soon! Previous data obtained {}s {:03}ms ago. ",
                    imu_elapsed.as_secs(),
                    imu_elapsed.subsec_millis()
                );
                false
            } else {
                eprintln!("Kalman: Time inversion");
                false
            }
        }
        Telemetry::Position(_data) => {true},
    }
}

pub fn create_matrix_A(dt: f64) -> Matrix6<f64> {
    Matrix6::new(
    1.0, 0.0, 0.0, dt,  0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0, dt,  0.0, 
    0.0, 0.0, 1.0, 0.0, 0.0, dt, 
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 
    )
}

pub fn create_matrix_B(dt: f64) -> Matrix6x3<f64> {
    Matrix6x3::new(
    dt*dt*0.5, 0.0, 0.0, 
    0.0, dt*dt*0.5, 0.0, 
    0.0, 0.0, dt*dt*0.5, 
    dt, 0.0, 0.0, 
    0.0, dt, 0.0, 
    0.0, 0.0, dt, 
    )
}

fn create_matrix_H() -> Matrix3x6<f64> {
    Matrix3x6::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    )
}

fn create_matrix_Q(dt: f64, sigma_acc: f64) -> Matrix6<f64> {
    let Q = Matrix6::new(
        dt.powi(4)/4.0, 0.0,            0.0,            dt.powi(3)/2.0, 0.0,            0.0, 
        0.0,            dt.powi(4)/4.0, 0.0,            0.0,            dt.powi(3)/2.0, 0.0, 
        0.0,            0.0,            dt.powi(4)/4.0, 0.0,            0.0,            dt.powi(3)/2.0, 
        dt.powi(3)/2.0, 0.0,            0.0,            dt.powi(2),     0.0,            0.0, 
        0.0,            dt.powi(3)/2.0, 0.0,            0.0,            dt.powi(2),     0.0, 
        0.0,            0.0,            dt.powi(3)/2.0, 0.0,            0.0,            dt.powi(2), 
    );
    Q*sigma_acc
}

fn create_matrix_R(sigma_gps: f64) -> Matrix3<f64> {
    let R = Matrix3::<f64>::identity_generic(Const::<3>, Const::<3>);
    R*sigma_gps
}


#[cfg(test)]
mod test {

    use std::{sync::mpsc, time::SystemTime};
    use std::time::Duration;

    use super::*;

    #[test]
    fn test_create_matrix_A() {
        let dt = 0.1;
        let A = create_matrix_A(dt);
        approx::assert_abs_diff_eq!(A[(0, 0)], 1.0);
        approx::assert_abs_diff_eq!(A[(0, 3)], dt);
    }

    #[test]
    fn test_create_matrix_B() {
        let dt = 0.1;
        let B = create_matrix_B(dt);
        approx::assert_abs_diff_eq!(B[(0, 0)], dt*dt*0.5);
        approx::assert_abs_diff_eq!(B[(0, 1)], 0.0);
    }

    #[test]
    fn test_create_matrix_H() {
        let H = create_matrix_H();
        approx::assert_abs_diff_eq!(H[(0, 0)], 1.0);
        approx::assert_abs_diff_eq!(H[(0, 1)], 0.0);
    }

    #[test]
    fn test_create_matrix_Q() {
        let dt = 0.1;
        let sigma = 1.0;
        let Q = create_matrix_Q(dt, sigma);
        approx::assert_abs_diff_eq!(Q[(0, 0)], dt.powi(4)/4.0 * sigma);
        approx::assert_abs_diff_eq!(Q[(0, 1)], 0.0);
    }

    #[test]
    fn test_create_matrix_R() {
        let sigma = 0.01;
        let R = create_matrix_R(sigma);
        approx::assert_abs_diff_eq!(R[(0, 0)], sigma);
        approx::assert_abs_diff_eq!(R[(0, 1)], 0.0);
    }

    #[test]
    fn test_KalmanData_init() {
        let kd : KalmanData = KalmanData::new();
        approx::assert_abs_diff_eq!(kd.P[(5,1)], 0.0);
        approx::assert_abs_diff_eq!(kd.x[5], 0.0);
    }

    #[test]
    fn test_max_expected_imu_interval() {
        let base = get_cycle_duration_f64(IMU_FREQ);
        let expected = base * (1.0 + KALMAN_TIMING_TOLERANCE);
        let duration = max_expected_imu_interval();
        approx::assert_abs_diff_eq!(duration.as_secs_f64(), expected);
    }

    #[test]
    fn test_min_expected_imu_interval() {
        let base = get_cycle_duration_f64(IMU_FREQ);
        let expected = base * (1.0 - KALMAN_TIMING_TOLERANCE);
        let duration = min_expected_imu_interval();
        approx::assert_abs_diff_eq!(duration.as_secs_f64(), expected);
    }

    #[test]
    fn telemetry_check_test_imu_correct_intervals() {
        let data = Data::new();
        let telemetry_from_imu: Telemetry = Telemetry::Acceleration(data);
        let mut last_imu_data_timestamp: SystemTime = SystemTime::now();

        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        assert!(telemetry_check(
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
        ));

        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        assert!(telemetry_check(
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
        ));
        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        assert!(telemetry_check(
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
        ));
    }

    #[test]
    fn telemetry_check_test_imu_not_sending_data() {
        let data = Data::new();
        let telemetry_from_gps: Telemetry = Telemetry::Position(data);
        let mut last_imu_data_timestamp: SystemTime = SystemTime::now();
        assert!(telemetry_check(
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
        ));

        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        assert!(telemetry_check(
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
        ));

        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        assert!(telemetry_check(
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
        ));
    }

    #[test]
    fn telemetry_check_imu_incorrect_intervals() {
        let data = Data::new();
        let telemetry_from_imu: Telemetry = Telemetry::Acceleration(data);
        let mut last_imu_data_timestamp: SystemTime = SystemTime::now();

        
        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        assert!(telemetry_check(
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
        ));

        assert!(!telemetry_check(
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
        ));

        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        assert!(telemetry_check(
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
        ));
    }

    #[test]
    fn test_KalmanFilter_run() {    
 
        let (tx_imu, input_rx) = mpsc::channel();
        let tx_gps = tx_imu.clone();
        
        let (tx_kalman, rx_from_kalman) = mpsc::channel();

        let transmitters : Vec<Sender<Telemetry>> = vec![tx_kalman]; 

        let kalman_handle = KalmanFilter::run(
            transmitters,
            input_rx,
        );

    // send IMU data
        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(matches!(rx_from_kalman.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));

        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(matches!(rx_from_kalman.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));
        
        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(matches!(rx_from_kalman.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));
        
    // send GPS data
        std::thread::sleep(Duration::from_secs_f64(1.0));
        let mut gps_data = Data { 
            x: 0.0, 
            y: 0.0, 
            z: 0.0, 
            timestamp: SystemTime::now().checked_sub(Duration::from_secs(1)).unwrap()
        };
        let mut telemetry_from_gps: Telemetry = Telemetry::Position(gps_data);

        let _ = tx_gps.send(telemetry_from_gps);
        assert!(matches!(rx_from_kalman.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));

        gps_data = Data { 
            x: 1.0, 
            y: 1.0, 
            z: 1.0, 
            timestamp: gps_data.timestamp.checked_add(Duration::from_secs(1)).unwrap()
        };
        telemetry_from_gps = Telemetry::Position(gps_data);

        let _ = tx_gps.send(telemetry_from_gps);
        assert!(matches!(rx_from_kalman.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));

    // Kalman should start Kalmaning

        // send zero acceleration and expect no change in position estimate ([1,1,1] - as set with gps data)
        // why is DT_IMU added to position data??? because 1(m/s) * DT_IMU(s) = DT_IMU(m) 
        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 0.0, y: 0.0, z: 0.0, timestamp: SystemTime::now() }));
        match rx_from_kalman.recv() {
            Ok(data) => {
                approx::assert_abs_diff_eq!(data.data().x, 1.0 + get_cycle_duration_f64(IMU_FREQ));
                approx::assert_abs_diff_eq!(data.data().y, 1.0 + get_cycle_duration_f64(IMU_FREQ));
                approx::assert_abs_diff_eq!(data.data().z, 1.0 + get_cycle_duration_f64(IMU_FREQ));
            },
            Err(e) => panic!("Failed to receive: {e}"),
        }
        
        let _ = tx_gps.send(Telemetry::Position(Data { 
            x: 1.0 + get_cycle_duration_f64(IMU_FREQ), 
            y: 1.0 + get_cycle_duration_f64(IMU_FREQ), 
            z: 1.0 + get_cycle_duration_f64(IMU_FREQ), 
            timestamp: SystemTime::now() }));
        match rx_from_kalman.recv() {
            Ok(data) => {
                approx::assert_abs_diff_eq!(data.data().x, 1.0 + get_cycle_duration_f64(IMU_FREQ));
                approx::assert_abs_diff_eq!(data.data().y, 1.0 + get_cycle_duration_f64(IMU_FREQ));
                approx::assert_abs_diff_eq!(data.data().z, 1.0 + get_cycle_duration_f64(IMU_FREQ));
            },
            Err(e) => panic!("Failed to receive: {e}"),
        }

        std::thread::sleep(Duration::from_secs(2));

        drop(tx_imu);
        drop(tx_gps);
        
        kalman_handle.join().unwrap();
    }
}

