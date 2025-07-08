#[allow(non_snake_case)]

use nalgebra::{Const, Matrix3, Matrix3x6, Matrix6, Matrix6x3};

// matrixROWSxCOLUMNS

pub struct KalmanFilter {
    A: nalgebra::Matrix6<f64>,
    B: nalgebra::Matrix6x3<f64>,
    H: nalgebra::Matrix3x6<f64>,
    Q: nalgebra::Matrix6<f64>,
    R: nalgebra::Matrix3<f64>,
    
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