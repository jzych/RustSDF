#![allow(non_snake_case)]
use std::{
    sync::mpsc::{Receiver, Sender},
    thread::JoinHandle,
    time::SystemTime,
};
use nalgebra::{Matrix3x1, Matrix6, Matrix6x1, Matrix6x3};
use super::initialize_state_using_gps_data;

use crate::{
    config::IMU_FREQ,
    data::{Data, Telemetry},
    log_config::{INTERTIAL_NAVIGATOR_LOG, GENERAL_LOG},
    logger::log,
    utils::*,
    kalman::{create_matrix_A, create_matrix_B},
};

pub struct InertialNavigator {
    tx: Vec<Sender<Telemetry>>,
    A: Matrix6<f64>,
    B: Matrix6x3<f64>,
    state: Matrix6x1<f64>,
}


impl InertialNavigator {
    fn new(tx: Vec<Sender<Telemetry>>) -> InertialNavigator {
        InertialNavigator {
            tx,
            A: create_matrix_A(get_cycle_duration_f64(IMU_FREQ)),
            B: create_matrix_B(get_cycle_duration_f64(IMU_FREQ)),
            state: Matrix6x1::new(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,),
        }
    }

    pub fn run(
        tx: Vec<Sender<Telemetry>>,
        rx: Receiver<Telemetry>,
    ) -> JoinHandle<()> {
        let mut inertial_navigator = InertialNavigator::new(tx);
        let mut gps_samples_received : u32 = 0;
        let mut prev_gps_data : Data = Data::new();

        std::thread::spawn( move || {
            for telemetry in &rx {
                initialize_state_using_gps_data(
                        telemetry,
                        &mut gps_samples_received,
                        &mut inertial_navigator.state,
                        &mut prev_gps_data,
                );
                if gps_samples_received == 2 {
                    break;
                }
            }
            for telemetry in rx {
                match telemetry {                          
                    Telemetry::Acceleration(data) => {
                        let u = Matrix3x1::new(data.x, data.y, data.z);
                        inertial_navigator.state = inertial_navigator.A * inertial_navigator.state + inertial_navigator.B * u;
                    }
                    Telemetry::Position(_data) => {},
                }
                
                let inertial_navigator_position_estimate = Telemetry::Position(Data {
                    x: inertial_navigator.state[0],
                    y: inertial_navigator.state[1],
                    z: inertial_navigator.state[2],
                    timestamp: SystemTime::now()
                });

                inertial_navigator.tx.retain(|tx| tx.send(inertial_navigator_position_estimate).is_ok());
                if inertial_navigator.tx.is_empty() {
                    break;
                }
                log(INTERTIAL_NAVIGATOR_LOG, inertial_navigator_position_estimate);

            }
            log(GENERAL_LOG, "Inertial navigator removed".to_string());
        })
    }   
}

#[cfg(test)]
mod test {

    use ntest_timeout::timeout;
    use std::{
        sync::mpsc::Sender,
        sync::mpsc,
        time::SystemTime,
    };

    use super::*;

    #[test]
    #[timeout(10000)]
    fn test_new_inertial_navigator() {
        let (tx, _rx) = mpsc::channel();
        let inertial_navigator = InertialNavigator::new(vec![tx]);
        approx::assert_abs_diff_eq!(inertial_navigator.A[(0,3)], get_cycle_duration_f64(IMU_FREQ));
        assert!(inertial_navigator.tx[0].send(Telemetry::Acceleration(Data::new())).is_ok());
    }

    #[test]
    #[timeout(10000)]
    fn test_inertial_navigator_run(){
        let (tx_imu, input_rx) = mpsc::channel();
        let tx_gps = tx_imu.clone();
        let (tx_inertial_nav, rx_inertial_nav) = mpsc::channel();

        let transmitters : Vec<Sender<Telemetry>> = vec![tx_inertial_nav]; 

        let inertial_nav_handle = InertialNavigator::run(
            transmitters,
            input_rx,
        );

        // send IMU data and expect nothing in Intertial Navigator's output channel since state is not initialized yet
        let _ = tx_imu.send(Telemetry::Position(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(matches!(rx_inertial_nav.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));

        // send GPS data and expect nothing in Intertial Navigator's output channel
        let _ = tx_gps.send(Telemetry::Position(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(matches!(rx_inertial_nav.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));

        // send GPS data and expect nothing in Intertial Navigator's output channel
        let _ = tx_gps.send(Telemetry::Position(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(matches!(rx_inertial_nav.try_recv(), Err(std::sync::mpsc::TryRecvError::Empty)));

        // send IMU data and expect something in Intertial Navigator's output channel
        let _ = tx_imu.send(Telemetry::Acceleration(Data { x: 1.0, y: 1.0, z: 1.0, timestamp: SystemTime::now() }));
        assert!(rx_inertial_nav.recv().is_ok());

        drop(tx_imu);
        drop(tx_gps);
        
        assert!(inertial_nav_handle.join().is_ok());
    }


}