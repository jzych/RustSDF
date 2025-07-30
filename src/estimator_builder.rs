use std::{
    sync::mpsc::Sender,
    thread::JoinHandle,
    sync::mpsc::{Receiver},
};
use crate::{
    average::Average,
    data::Telemetry,
    kalman::KalmanFilter,
    inertial_navigator::InertialNavigator,
};

#[derive(PartialEq, Eq, Debug)]
enum EstimatorType {
    Average,
    Kalman,
    InertialNavigator,
}

pub struct EstimatorBuilder {
    estimator_type: EstimatorType,
    subscribers: Vec<Sender<Telemetry>>,
    input_rx_option: Option<Receiver<Telemetry>>,
    buffer_length_option: Option<usize>,
}

impl EstimatorBuilder {
    fn default() -> Self {        
        Self {
            estimator_type: EstimatorType::Average,
            subscribers: Vec::new(),
            input_rx_option: None,   
            buffer_length_option: None, 
        }
    }

    pub fn new_average(buffer_length : usize) -> Self {
        Self {
            estimator_type: EstimatorType::Average,
            buffer_length_option: Some(buffer_length),
            ..Self::default()
        }
    }

    pub fn new_kalman() -> Self {
        Self {
            estimator_type: EstimatorType::Kalman,
            ..Self::default()
        }
    }

    pub fn new_inertial_navigator() -> Self {
        Self {
            estimator_type: EstimatorType::InertialNavigator,
            ..Self::default()
        }
    }

    pub fn with_subscribers(self, subscribers: Vec<Sender<Telemetry>>) -> Self {
        Self {
            subscribers,
            ..self
        }
    }

    pub fn with_input_rx(self, input_rx: Receiver<Telemetry>) -> Self {
        let input_rx_option = Some(input_rx);
        Self {
            input_rx_option,
            ..self
        }
    }

    pub fn spawn(self) -> JoinHandle<()> {
        match self.input_rx_option {
            Some(input_rx) => {
                match self.estimator_type {
                    EstimatorType::Average => Average::run(
                        self.subscribers,
                        input_rx,
                        self.buffer_length_option.expect("Buffer length must be defined!"),
                    ),
                    EstimatorType::Kalman => KalmanFilter::run(
                        self.subscribers,
                        input_rx,
                    ),
                    EstimatorType::InertialNavigator => InertialNavigator::run(
                        self.subscribers,
                        input_rx,
                    ),
                }
            },
            None => panic!("Estimator Builder: Estimator with no receiving end tried to spawn!"),
        }

    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ntest_timeout::timeout;

    use crate::data::Data;

    #[test]
    fn expect_default_provides_estimator_type_average_with_no_subscribers() {
        let average_config = EstimatorBuilder::default();
        assert_eq!(average_config.estimator_type, EstimatorType::Average);
        assert!(average_config.subscribers.is_empty());
    }

    #[test]
    fn given_new_average_expect_builder_with_estimator_type_average() {
        let average_config = EstimatorBuilder::new_average(3 as usize);
        assert_eq!(average_config.estimator_type, EstimatorType::Average);
        assert!(average_config.subscribers.is_empty());
    }

    #[test]
    fn given_new_kalman_expect_builder_with_estimator_type_kalman() {
        let average_config = EstimatorBuilder::new_kalman();
        assert_eq!(average_config.estimator_type, EstimatorType::Kalman);
        assert!(average_config.subscribers.is_empty());
    }

    #[test]
    fn given_subscribers_expect_builder_with_provided_transmitters() {
        let (tx_1, _) = std::sync::mpsc::channel();
        let (tx_2, _) = std::sync::mpsc::channel();
        let builder_cfg = EstimatorBuilder::default().with_subscribers(vec![tx_1, tx_2]);
        assert_eq!(builder_cfg.subscribers.len(), 2);
    }

    #[test]
    #[timeout(10000)]
    #[should_panic]
    fn given_no_input_rx_expect_panic_on_spawn() {
        let _ = 
            EstimatorBuilder::new_kalman()
            .spawn();
    }

    #[test]
    #[timeout(10000)]
    fn given_input_rx_expect_builder_with_set_input_rx() {
        let (tx, input_rx) = std::sync::mpsc::channel();
        let builder_cfg = EstimatorBuilder::default().with_input_rx(input_rx);
        tx.send(Telemetry::Acceleration(Data::new())).unwrap();
        assert!(builder_cfg.input_rx_option.unwrap().recv().is_ok());
    }

    #[test]
    #[timeout(10000)]
    fn given_average_builder_expect_spawn_to_spawn_average_thread() {
        let (_, input_rx) = std::sync::mpsc::channel();
        let handle = EstimatorBuilder::new_average(3 as usize)
            .with_input_rx(input_rx)
            .spawn();
        assert!(handle.join().is_ok());
    }

    #[test]
    #[should_panic]
    fn given_no_buffer_length_expect_panic_on_avereage_filter_creation() {
        let (_, input_rx) = std::sync::mpsc::channel();
        let mut builder = EstimatorBuilder::new_average(3 as usize)
            .with_input_rx(input_rx);
        builder.buffer_length_option = None;
        let _ = builder.spawn();
    }

    #[test]
    #[timeout(10000)]
    fn given_kalman_builder_expect_spawn_to_spawn_kalman_thread() {
        let (_, input_rx) = std::sync::mpsc::channel();
        let handle = EstimatorBuilder::new_kalman()
            .with_input_rx(input_rx)
            .spawn();
        assert!(handle.join().is_ok());
    }
    
    #[test]
    #[timeout(10000)]
    fn given_inertial_nav_builder_expect_spawn_to_spawn_inertial_nav_thread() {
        let (_, input_rx) = std::sync::mpsc::channel();
        let handle = EstimatorBuilder::new_inertial_navigator()
            .with_input_rx(input_rx)
            .spawn();
        assert!(handle.join().is_ok());
    }
}
