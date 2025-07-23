use std::{
    num::NonZeroU32,
    sync::{atomic::AtomicBool, mpsc::Sender, Arc, Mutex},
    thread::JoinHandle,
};

use crate::{
    data::{Data, Telemetry},
    gps::Gps,
    imu::Imu,
};

#[derive(PartialEq, Eq, Debug)]
enum ProviderType {
    Gps,
    Imu,
}

pub struct SensorBuilder {
    provider_type: ProviderType,
    frequency: NonZeroU32,
    transmitters: Vec<Sender<Telemetry>>,
    position_generator: Arc<Mutex<Data>>,
    noise_standard_deviation: f64,
}

impl SensorBuilder {
    fn default() -> Self {
        Self {
            provider_type: ProviderType::Imu,
            frequency: NonZeroU32::new(1).unwrap(),
            transmitters: Vec::new(),
            position_generator: Arc::new(Mutex::new(Data::new())),
            noise_standard_deviation: 0.0,
        }
    }

    pub fn new_imu() -> Self {
        Self {
            provider_type: ProviderType::Imu,
            ..Self::default()
        }
    }

    pub fn new_gps() -> Self {
        Self {
            provider_type: ProviderType::Gps,
            ..Self::default()
        }
    }

    pub fn with_frequency(self, frequency: NonZeroU32) -> Self {
        Self {
            frequency,
            ..self
        }
    }

    pub fn with_position_generator(self, position_generator: Arc<Mutex<Data>>) -> Self {
        Self {
            position_generator,
            ..self
        }
    }

    pub fn with_subscribers(self, transmitters: Vec<Sender<Telemetry>>) -> Self {
        Self {
            transmitters,
            ..self
        }
    }

    pub fn with_noise(self, noise_standard_deviation: f64) -> Self {
        Self {
            noise_standard_deviation,
            ..self
        }
    }

    pub fn spawn(self, shutdown: Arc<AtomicBool>) -> JoinHandle<()> {
        match self.provider_type {
            ProviderType::Imu => Imu::run(
                self.position_generator,
                self.transmitters,
                shutdown,
                self.frequency,
            ),
            ProviderType::Gps => Gps::run(
                self.position_generator,
                self.transmitters,
                shutdown,
                self.frequency,
                self.noise_standard_deviation,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ntest_timeout::timeout;

    #[test]
    fn expect_default_provides_valid_imu_config() {
        let expected_freq = NonZeroU32::new(1).unwrap();
        let imu_config = SensorBuilder::default();
        assert_eq!(imu_config.frequency, expected_freq);
        assert!(imu_config.transmitters.is_empty());
    }

    #[test]
    fn given_new_imu_expect_builder_with_imu_as_signal_provider() {
        let builder_cfg = SensorBuilder::new_imu();
        assert_eq!(builder_cfg.provider_type, ProviderType::Imu);
    }

    #[test]
    fn given_new_gps_expect_builder_with_gps_as_signal_provider() {
        let builder_cfg = SensorBuilder::new_gps();
        assert_eq!(builder_cfg.provider_type, ProviderType::Gps);
    }

    #[test]
    fn given_new_frequency_expect_builder_with_set_frequency() {
        let frequency = NonZeroU32::new(5).unwrap();
        let builder_cfg = SensorBuilder::default().with_frequency(frequency);
        assert_eq!(builder_cfg.frequency, frequency);
    }

    #[test]
    fn given_subscribers_expect_builder_with_provided_senders() {
        let (tx_1, _) = std::sync::mpsc::channel();
        let (tx_2, _) = std::sync::mpsc::channel();
        let builder_cfg = SensorBuilder::default().with_subscribers(vec![tx_1, tx_2]);
        assert_eq!(builder_cfg.transmitters.len(), 2);
    }

    #[test]
    fn given_position_generator_expect_builder_with_provided_generator() {
        let position_generator = Arc::new(Mutex::new(Data::new()));
        let builder_cfg = SensorBuilder::default()
            .with_position_generator(Arc::clone(&position_generator));
        let expected_x = 1.0;
        let expected_y = 2.0;
        let expected_z = 3.0;
        {
            let mut pos = position_generator.lock().unwrap();
            pos.x = expected_x;
            pos.y = expected_y;
            pos.z = expected_z;
        }
        let position_data = builder_cfg.position_generator.lock().unwrap();
        approx::assert_abs_diff_eq!(position_data.x, expected_x);
        approx::assert_abs_diff_eq!(position_data.y, expected_y);
        approx::assert_abs_diff_eq!(position_data.z, expected_z);
    }

    #[test]
    #[timeout(10000)]
    fn given_imu_builder_expect_spawn_to_start_imu() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (tx, rx) = std::sync::mpsc::channel();
        let handle = SensorBuilder::new_imu()
            .with_subscribers(vec![tx])
            .spawn(Arc::clone(&shutdown));

        let Telemetry::Acceleration(_) = rx.recv().unwrap() else {
            panic!("IMU should provide acceleration. Got position.")
        };
        shutdown.store(true, std::sync::atomic::Ordering::SeqCst);
        handle.join().unwrap();
    }

    #[test]
    #[timeout(10000)]
    fn given_gps_builder_expect_spawn_to_start_gps() {
        let shutdown = Arc::new(AtomicBool::new(false));
        let (tx, rx) = std::sync::mpsc::channel();
        let handle = SensorBuilder::new_gps()
            .with_subscribers(vec![tx])
            .spawn(Arc::clone(&shutdown));

        let Telemetry::Position(_) = rx.recv().unwrap() else {
            panic!("GPS should provide position. Got acceleration.")
        };
        shutdown.store(true, std::sync::atomic::Ordering::SeqCst);
        handle.join().unwrap();
    }
}
