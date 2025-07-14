#![allow(non_snake_case)]

use nalgebra::{Const, Matrix3, Matrix3x6, Matrix6, Matrix6x3, Matrix6x1, Matrix3x1};
use std::thread::JoinHandle;
use crate::data::{Data, Telemetry};
use std::time::SystemTime;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};




const DT_IMU : f64 = 0.1; // seconds
// const KALMAN_INTERVAL : u64 = DT_IMU as u64 * 1000; // miliseconds
const SIGMA_ACC : f64 = 0.01;
const SIGMA_GPS : f64 = 0.1;


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
    previous_kalman_state: KalmanData,
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
            previous_kalman_state: KalmanData::new(),
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
        shutdown: Arc<AtomicBool>,
    ) -> JoinHandle<()> {
        std::thread::spawn( move || {
            let mut state: KalmanData = KalmanData::new();
            let mut kalman = KalmanFilter::new(tx);
            let mut kalman_position_estimate : Telemetry;
            while !shutdown.load(Ordering::SeqCst) {
                for telemetry in rx.iter() {
                    println!(
                        "Kalman consumer: received: {}, {}, {}",
                        telemetry.data().x, telemetry.data().y, telemetry.data().z
                    );
                    match telemetry {
                        
                        Telemetry::Acceleration(data) => {
                            // prediction
                            let u = Matrix3x1::new(data.x, data.y, data.z);
                            state.x = kalman.A * kalman.previous_kalman_state.x + kalman.B * u;
                            state.P = kalman.A * kalman.previous_kalman_state.P * kalman.A.transpose() + kalman.Q;
                        }
                        Telemetry::Position(data) => {
                            // correction
                            let z = Matrix3x1::new(data.x, data.y, data.z);
                            let K = state.P * kalman.H.transpose() * (kalman.H * state.P * kalman.H.transpose() + kalman.R).try_inverse().unwrap();
                            state.x = state.x + K * (z - kalman.H * state.x);
                            state.P = (Matrix6::identity_generic(Const::<6>,Const::<6>) - K * kalman.H) * state.P;
                        },
                    }

                    // println!("Current state estimate: {}", state.x);
                    // println!("Current prob matrix: {}", state.P);
                    kalman.previous_kalman_state = state;
                    kalman_position_estimate = Telemetry::Position(Data { x: state.x[0], y: state.x[1], z: state.x[2], timestamp: SystemTime::now() });
                    println!(
                        "Kalman is sending: {}, {}, {}",
                        kalman_position_estimate.data().x, kalman_position_estimate.data().y, kalman_position_estimate.data().z
                    );
                    kalman.tx.retain(|tx| tx.send(kalman_position_estimate).is_ok());
                    if kalman.tx.is_empty() {
                        break;
                    }
                }
            }
            println!("Kalman filter removed");
        })
        
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
    fn test_KalmanFilter_run() {
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        
        let (tx_imu, input_rx) = mpsc::channel();
        let tx_gps = tx_imu.clone();
        
        let (tx_kalman, rx_from_kalman) = mpsc::channel();

        let transmitters : Vec<Sender<Telemetry>> = vec![tx_kalman]; 

        let kalman_handle = KalmanFilter::run(
            transmitters,
            input_rx,
            Arc::clone(&shutdown_trigger),
        );


        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        match rx_from_kalman.recv() {
            Ok(data) => assert_ne!(data.data().x, 0.0),
            Err(e) => panic!("Failed to receive: {}", e),
        }

        let _ = tx_gps.send(Telemetry::Position(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        
        match rx_from_kalman.recv() {
            Ok(data) => assert_ne!(data.data().x, 0.0),
            Err(e) => panic!("Failed to receive: {}", e),
        }

        std::thread::sleep(Duration::from_secs(2));
        shutdown_trigger.store(true, Ordering::SeqCst);

        drop(tx_imu);
        drop(tx_gps);

        kalman_handle.join().unwrap();
    }
    


}

