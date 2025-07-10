#![allow(non_snake_case)]

use nalgebra::{Const, Matrix3, Matrix3x6, Matrix6, Matrix6x3, Matrix6x1, Matrix3x1};
use std::thread::JoinHandle;
use crate::data::{Data, Telemetry};
use std::time::{Duration, Instant, SystemTime};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};



const DT_IMU : f64 = 0.1; // seconds
const KALMAN_INTERVAL : u64 = DT_IMU as u64 * 1000; // miliseconds
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
    A: Matrix6<f64>,
    B: Matrix6x3<f64>,
    H: Matrix3x6<f64>,
    Q: Matrix6<f64>,
    R: Matrix3<f64>,
    previous_kalman_state: KalmanData,
}

impl KalmanFilter {

    pub fn new() -> Self {
        
        Self {
            A: create_matrix_A(DT_IMU),
            B: create_matrix_B(DT_IMU),
            H: create_matrix_H(),
            Q: create_matrix_Q(DT_IMU, SIGMA_ACC),
            R: create_matrix_R(SIGMA_GPS),
            previous_kalman_state: KalmanData::new(),
        }
    }

    pub fn show(&self){
        println!("A: {}", self.A);
        println!("B: {}", self.B);
        println!("H: {}", self.H);
        println!("Q: {}", self.Q);
        println!("R: {}", self.R);
    }

    pub fn compute(&mut self, input: Telemetry) -> KalmanData {
        
        let mut state: KalmanData = KalmanData::new();
        
        match input {
            Telemetry::Acceleration(data) => {
                // prediction
                let u = Matrix3x1::new(data.x, data.y, data.z);
                state.x = self.A * self.previous_kalman_state.x + self.B * u;
                state.P = self.A * self.previous_kalman_state.P * self.A.transpose() + self.Q;
            }
            Telemetry::Position(data) => {
                // correction
                let z = Matrix3x1::new(data.x, data.y, data.z);
                let K = state.P * self.H.transpose() * (self.H * state.P * self.H.transpose() + self.R).try_inverse().unwrap();
                state.x = state.x + K * (z - self.H * state.x);
                state.P = (Matrix6::identity_generic(Const::<6>,Const::<6>) - K * self.H) * state.P;
            },
        }

        // println!("Current state estimate: {}", state.x);
        // println!("Current prob matrix: {}", state.P);
        self.previous_kalman_state = state;

        state
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

    use super::*;

    #[test]
    fn test_create_matrix_A() {
        let dt = 0.1;
        let A = create_matrix_A(dt);
        assert_eq!(A[(0, 0)], 1.0);
        assert_eq!(A[(0, 3)], dt);
    }

    #[test]
    fn test_kalman_filter_initialization() {
        let kf = KalmanFilter::new();
        assert_eq!(kf.previous_kalman_state.x, Matrix6x1::zeros());
    }

    #[test]
    fn test_prediction_step() {
        let mut kf = KalmanFilter::new();
        let acc = Data { x: 1.0, y: 0.0, z: 0.0, timestamp: SystemTime::now() };
        let telemetry = Telemetry::Acceleration(acc);
        let result = kf.compute(telemetry);
        assert_ne!(result.x, Matrix6x1::zeros());
    }
   
    #[test]
    fn test_correction_step() {
        let mut kf = KalmanFilter::new();
        // First do a prediction to initialize state
        let acc = Data { x: 0.0, y: 0.0, z: 0.0, timestamp: SystemTime::now() };
        kf.compute(Telemetry::Acceleration(acc));

        let gps = Data { x: 5.0, y: 5.0, z: 5.0, timestamp: SystemTime::now() };
        let result = kf.compute(Telemetry::Position(gps));
        assert_ne!(result.x, Matrix6x1::zeros());
    }



}

