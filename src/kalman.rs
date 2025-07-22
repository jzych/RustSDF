#![allow(non_snake_case)]

use std::{
    sync::mpsc::{Receiver, Sender},
    thread::JoinHandle,
    time::{Duration, SystemTime},
};
use nalgebra::{Const, Matrix3, Matrix3x6, Matrix3x1, Matrix6, Matrix6x1, Matrix6x3};
use crate::{
    data::{Data, Telemetry},
    logger::log,
};



const DT_IMU : f64 = 0.05; // seconds
const TIMING_TOLERANCE : f64 = 0.02; // 0.01 = 1% of timing tolerance
const SIGMA_ACC : f64 = 0.1;
const SIGMA_GPS : f64 = 0.1;
const LOGGER_PREFIX: &str = "KALMAN";


#[derive(Debug, Copy, Clone)]
pub struct KalmanData {
    x: Matrix6x1<f64>,
    P: Matrix6<f64>,
}

impl KalmanData {
    pub fn new() -> Self{
        Self {
            x: Matrix6x1::zeros_generic(Const::<6>, Const::<1>),
            P: Matrix6::zeros_generic(Const::<6>, Const::<6>),            
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
            A: create_matrix_A(DT_IMU),
            B: create_matrix_B(DT_IMU),
            H: create_matrix_H(),
            Q: create_matrix_Q(DT_IMU, SIGMA_ACC),
            R: create_matrix_R(SIGMA_GPS),
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
        let imu_samples_to_skip : u32 = 0;
        let mut imu_samples_received : u32 = 0;
        let mut last_imu_data_timestamp = SystemTime::now();
        let mut gps_samples_received : u32 = 0;
        let mut prev_gps_data : Data = Data::new();

        std::thread::spawn( move || {
            

            for telemetry in rx {

                if telemetry_check(
                    imu_samples_to_skip,
                    &mut imu_samples_received,
                    telemetry,
                    &mut last_imu_data_timestamp,
                    &mut gps_samples_received,
                    &mut kalman.state,
                    &mut prev_gps_data
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
                    log(LOGGER_PREFIX, kalman_position_estimate);
                }
            }
            println!("Kalman filter removed");
        })
    }   
}


fn telemetry_check(
    imu_samples_to_skip: u32,
    imu_samples_received: &mut u32,
    telemetry: Telemetry,
    last_imu_data_timestamp: &mut SystemTime,
    gps_samples_received: &mut u32,
    state: &mut KalmanData,
    prev_gps_data: &mut Data,
) -> bool {
    match telemetry {                    
        Telemetry::Acceleration(_) => {
            let current_imu_data_timestamp = SystemTime::now();
            let imu_elapsed: Duration = current_imu_data_timestamp.duration_since(*last_imu_data_timestamp).unwrap(); 
            *last_imu_data_timestamp = current_imu_data_timestamp;
            *imu_samples_received += 1;

            if (*imu_samples_received <= imu_samples_to_skip) || (*gps_samples_received < 2) {
                false
            } else if imu_elapsed > Duration::from_secs_f64(DT_IMU * (1.0 + TIMING_TOLERANCE)) {
                eprintln!("Kalman: IMU data is late! Previous data obtained {}s {:03}ms ago. ",
                    imu_elapsed.as_secs(),
                    imu_elapsed.subsec_millis()
                );
                true
            } else if imu_elapsed >= Duration::from_secs_f64(DT_IMU * (1.0 - TIMING_TOLERANCE)) {
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
        Telemetry::Position(data) => {
            *gps_samples_received += 1;
            
            if *gps_samples_received < 2 {
                *prev_gps_data = data;
                false
            } else if *gps_samples_received == 2 {
                let delta_time = data.timestamp.duration_since(prev_gps_data.timestamp).unwrap().as_secs_f64();
                state.x[0] = data.x;
                state.x[1] = data.y;
                state.x[2] = data.z;
                state.x[3] = (data.x - prev_gps_data.x)/delta_time;
                state.x[4] = (data.y - prev_gps_data.y)/delta_time;
                state.x[5] = (data.z - prev_gps_data.z)/delta_time;
                println!("Kalman: Initial position from GPS data: {}, {}, {}",
                    state.x[0],
                    state.x[1],
                    state.x[2]
                );
                println!("Kalman: Initial velocity from GPS data: {}, {}, {}, dt = {}",
                    state.x[3],
                    state.x[4],
                    state.x[5],
                    delta_time,
                );
                false
            } else {
                *imu_samples_received > imu_samples_to_skip
            }
        }
    }
}

fn create_matrix_A(dt: f64) -> Matrix6<f64> {
    Matrix6::new(
    1.0, 0.0, 0.0, dt,  0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0, dt,  0.0, 
    0.0, 0.0, 1.0, 0.0, 0.0, dt, 
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 
    )
}

fn create_matrix_B(dt: f64) -> Matrix6x3<f64> {
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
    use nalgebra::Matrix6x1;

    use super::*;

    #[test]
    fn test_create_matrix_A() {
        let dt = 0.1;
        let A = create_matrix_A(dt);
        assert_eq!(A[(0, 0)], 1.0);
        assert_eq!(A[(0, 3)], dt);
    }

    #[test]
    fn test_create_matrix_B() {
        let dt = 0.1;
        let B = create_matrix_B(dt);
        assert_eq!(B[(0, 0)], dt*dt*0.5);
        assert_eq!(B[(0, 1)], 0.0);
    }

    #[test]
    fn test_create_matrix_H() {
        let H = create_matrix_H();
        assert_eq!(H[(0, 0)], 1.0);
        assert_eq!(H[(0, 1)], 0.0);
    }

    #[test]
    fn test_create_matrix_Q() {
        let dt = 0.1;
        let sigma = 1.0;
        let Q = create_matrix_Q(dt, sigma);
        assert_eq!(Q[(0, 0)], dt.powi(4)/4.0 * sigma);
        assert_eq!(Q[(0, 1)], 0.0);
    }

    #[test]
    fn test_create_matrix_R() {
        let sigma = 0.01;
        let R = create_matrix_R(sigma);
        assert_eq!(R[(0, 0)], sigma);
        assert_eq!(R[(0, 1)], 0.0);
    }

    #[test]
    fn test_KalmanData_init() {
        let kd : KalmanData = KalmanData::new();
        assert_eq!(kd.P[(5,5)], 0.0);
        assert_eq!(kd.x[5], 0.0);
    }

    #[test]
    fn telemetry_check_test_gps_not_sending_data() {
        let imu_samples_to_skip: u32 = 2;
        let mut imu_samples_received: u32 = 0;
        let mut gps_samples_received: u32 = 0;
        let data = Data::new();
        let telemetry_from_imu: Telemetry = Telemetry::Acceleration(data);
        let mut last_imu_data_timestamp: SystemTime = SystemTime::now();
        let mut state: KalmanData = KalmanData::new();
        let mut prev_gps_data: Data = Data::new();

        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));
    }

    #[test]
    fn telemetry_check_test_imu_not_sending_data() {
        let imu_samples_to_skip: u32 = 2;
        let mut imu_samples_received: u32 = 0;
        let mut gps_samples_received: u32 = 0;
        let data = Data::new();
        let telemetry_from_gps: Telemetry = Telemetry::Position(data);
        let mut last_imu_data_timestamp: SystemTime = SystemTime::now();
        let mut state: KalmanData = KalmanData::new();
        let mut prev_gps_data: Data = Data::new();

        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));
    }

    #[test]
    fn telemetry_check_happy_path_test() {
        let imu_samples_to_skip: u32 = 2;
        let mut imu_samples_received: u32 = 0;
        let mut gps_samples_received: u32 = 0;
        let mut last_imu_data_timestamp: SystemTime = SystemTime::now();
        let mut state: KalmanData = KalmanData::new();
        let mut prev_gps_data: Data = Data::new();

        let mut gps_data = Data { 
            x: 0.0, 
            y: 0.0, 
            z: 0.0, 
            timestamp: SystemTime::now().checked_sub(Duration::from_secs(1)).unwrap()
        };
        let mut telemetry_from_gps: Telemetry = Telemetry::Position(gps_data);
        
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        gps_data = Data { 
            x: 1.0, 
            y: 1.0, 
            z: 1.0, 
            timestamp: gps_data.timestamp.checked_add(Duration::from_secs(1)).unwrap()
        };
        telemetry_from_gps = Telemetry::Position(gps_data);
        
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));
        assert_eq!(state.x, Matrix6x1::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
         
        let imu_data = Data::new();
        let telemetry_from_imu = Telemetry::Acceleration(imu_data);

        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        // make sure GPS is not served before IMU (prediction should run before correction)
        assert!(!telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));

        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        assert!(telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_imu,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        ));
        assert!(telemetry_check(
            imu_samples_to_skip,
            &mut imu_samples_received,
            telemetry_from_gps,
            &mut last_imu_data_timestamp,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
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

        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(matches!(rx_from_kalman.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));
        
        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
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
        std::thread::sleep(Duration::from_secs_f64(DT_IMU));
        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 0.0, y: 0.0, z: 0.0, timestamp: SystemTime::now() }));
        match rx_from_kalman.recv() {
            Ok(data) => {
                assert_eq!(data.data().x, 1.0 + DT_IMU);
                assert_eq!(data.data().y, 1.0 + DT_IMU);
                assert_eq!(data.data().z, 1.0 + DT_IMU);
            },
            Err(e) => panic!("Failed to receive: {e}"),
        }
        
        let _ = tx_gps.send(Telemetry::Position(Data { x: 1.0 + DT_IMU, y: 1.0 + DT_IMU, z: 1.0 + DT_IMU, timestamp: SystemTime::now() }));
        match rx_from_kalman.recv() {
            Ok(data) => {
                assert_eq!(data.data().x, 1.0 + DT_IMU);
                assert_eq!(data.data().y, 1.0 + DT_IMU);
                assert_eq!(data.data().z, 1.0 + DT_IMU);
            },
            Err(e) => panic!("Failed to receive: {e}"),
        }

        std::thread::sleep(Duration::from_secs(2));

        drop(tx_imu);
        drop(tx_gps);
        
        kalman_handle.join().unwrap();
    }
}

