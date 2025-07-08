#[allow(non_snake_case)]

use nalgebra::{Const, Matrix3, Matrix3x6, Matrix6, Matrix6x3, Matrix6x1, Matrix3x1};

use crate::data; 

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
    
}

impl KalmanFilter {

    pub fn new(dt: f64, sigma_acc: f64, sigma_gps: f64) -> Self {
        
        Self {
            A: create_matrix_A(dt),
            B: create_matrix_B(dt),
            H: create_matrix_H(),
            Q: create_matrix_Q(dt, sigma_acc),
            R: create_matrix_R(sigma_gps),}
    }

    pub fn show(&self){
        println!("A: {}", self.A);
        println!("B: {}", self.B);
        println!("H: {}", self.H);
        println!("Q: {}", self.Q);
        println!("R: {}", self.R);
    }

    pub fn compute(&self, prev_state: KalmanData, input_acc: data::Data, input_gps: data::Data) -> KalmanData {
        let u = Matrix3x1::new(input_acc.x, input_acc.y, input_acc.z);
        let z = Matrix3x1::new(input_gps.x, input_gps.y, input_gps.z);

        let mut state = KalmanData::new();

        // prediction
        state.x = self.A * prev_state.x + self.B * u;
        state.P = self.A * prev_state.P * self.A.transpose() + self.Q;

        // correction
        let K = state.P * self.H.transpose() * (self.H * state.P * self.H.transpose() + self.R).try_inverse().unwrap();
        state.x = state.x + K * (z - self.H * state.x);
        state.P = (Matrix6::identity_generic(Const::<6>,Const::<6>) - K * self.H) * state.P;

        println!("Current state estimate: {}", state.x);
        println!("Current prob matrix: {}", state.P);

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