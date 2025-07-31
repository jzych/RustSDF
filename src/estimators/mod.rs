use nalgebra::Matrix6x1;

use crate::{
    data::{Data, Telemetry},
};

pub mod kalman;
pub mod inertial_navigator;

pub fn initialize_state_using_gps_data(
    telemetry: Telemetry,
    gps_samples_received: &mut u32,
    state: &mut Matrix6x1<f64>,
    prev_gps_data: &mut Data,
) {
    match telemetry {                          
        Telemetry::Acceleration(_data) => {},
        Telemetry::Position(data) => {
            *gps_samples_received += 1;
            
            if *gps_samples_received == 1 {
                *prev_gps_data = data;
            } else {
                let delta_time = data.timestamp.duration_since(prev_gps_data.timestamp).unwrap().as_secs_f64();
                state[0] = data.x;
                state[1] = data.y;
                state[2] = data.z;
                state[3] = (data.x - prev_gps_data.x)/delta_time;
                state[4] = (data.y - prev_gps_data.y)/delta_time;
                state[5] = (data.z - prev_gps_data.z)/delta_time;
            }
        },
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use ntest_timeout::timeout;
    use std::time::{Duration, SystemTime};

use crate::{
    data::{Data, Telemetry},
    utils::get_cycle_duration,
    config::IMU_FREQ,
};
    
    #[test]
    #[timeout(10000)]
    fn initialize_state_using_gps_data_test_gps_not_sending_data() {
        let mut gps_samples_received: u32 = 0;
        let data = Data::new();
        let telemetry_from_imu: Telemetry = Telemetry::Acceleration(data);
        let mut state = Matrix6x1::new(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,);
        let mut prev_gps_data: Data = Data::new();

        initialize_state_using_gps_data(
            telemetry_from_imu,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        );
        assert_eq!(gps_samples_received, 0);
        assert_eq!(state, Matrix6x1::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

        std::thread::sleep(get_cycle_duration(IMU_FREQ));
        initialize_state_using_gps_data(
            telemetry_from_imu,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        );

        assert_eq!(gps_samples_received, 0);
        assert_eq!(state, Matrix6x1::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    #[test]
    #[timeout(10000)]
    fn initialize_state_using_gps_data_happy_path_test() {
        let mut gps_samples_received: u32 = 0;
        let mut state = Matrix6x1::new(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,);
        let mut prev_gps_data: Data = Data::new();

        let mut gps_data = Data { 
            x: 0.0, 
            y: 0.0, 
            z: 0.0, 
            timestamp: SystemTime::now().checked_sub(Duration::from_secs(1)).unwrap()
        };
        let mut telemetry_from_gps: Telemetry = Telemetry::Position(gps_data);
        
        // first GPS data
        initialize_state_using_gps_data(
            telemetry_from_gps,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        );
        assert_eq!(gps_samples_received, 1);
        assert_eq!(state, Matrix6x1::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

        gps_data = Data { 
            x: 1.0, 
            y: 1.0, 
            z: 1.0, 
            timestamp: gps_data.timestamp.checked_add(Duration::from_secs(1)).unwrap()
        };
        telemetry_from_gps = Telemetry::Position(gps_data);

        // second GPS data
        initialize_state_using_gps_data(
            telemetry_from_gps,
            &mut gps_samples_received,
            &mut state,
            &mut prev_gps_data
        );

        assert_eq!(gps_samples_received, 2);
        assert_eq!(state, Matrix6x1::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
    }
}
