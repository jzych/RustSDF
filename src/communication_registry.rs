use crate::data::Telemetry;
use std::{
    collections::HashMap,
    sync::mpsc::Sender,
};

#[allow(unused)]
#[derive(Copy, Clone, Debug, PartialEq, Hash, Eq)]
pub enum DataSource {
    Imu,
    Gps,
    Kalman,
    Average,
}

pub struct CommunicationRegistry {
    transmitter_registry: HashMap<DataSource, Vec<Sender<Telemetry>>>,
}

impl CommunicationRegistry {
    pub fn new() -> Self {
        Self {
            transmitter_registry: HashMap::new(),
        }
    }

    pub fn register_for_input(&mut self, source: DataSource, transmitter: Sender<Telemetry>) {
        self.transmitter_registry
            .entry(source)
            .or_default()
            .push(transmitter);
    }

    pub fn get_registered_transmitters(
        &mut self,
        source: DataSource,
    ) -> Option<Vec<Sender<Telemetry>>> {
        self.transmitter_registry.remove(&source)
    }
}

impl Default for CommunicationRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    #[test]
    fn new_returns_empty_registry() {
        let mut registry = CommunicationRegistry::new();
        assert!(registry.get_registered_transmitters(DataSource::Average).is_none());
        assert!(registry.get_registered_transmitters(DataSource::Gps).is_none());
        assert!(registry.get_registered_transmitters(DataSource::Kalman).is_none());
        assert!(registry.get_registered_transmitters(DataSource::Imu).is_none());
    }

    #[test]
    fn default_returns_empty_registry() {
        let mut registry = CommunicationRegistry::default();
        assert!(registry.get_registered_transmitters(DataSource::Average).is_none());
        assert!(registry.get_registered_transmitters(DataSource::Gps).is_none());
        assert!(registry.get_registered_transmitters(DataSource::Kalman).is_none());
        assert!(registry.get_registered_transmitters(DataSource::Imu).is_none());
    }

    #[test]
    fn given_one_subscriber_expect_one_transmitter_provided_along_with_ownership() {
        let (tx, _) = mpsc::channel();
        let mut registry = CommunicationRegistry::new();
        registry.register_for_input(DataSource::Imu, tx.clone());
        registry.register_for_input(DataSource::Gps, tx);
        
        assert_eq!(registry.get_registered_transmitters(DataSource::Imu).unwrap().len(), 1);
        assert!(registry.get_registered_transmitters(DataSource::Imu).is_none());
    }

    #[test]
    fn given_multiple_subscribers_expect_multiple_transmitters_provided() {
        let (tx, _) = mpsc::channel();
        let mut registry = CommunicationRegistry::new();
        registry.register_for_input(DataSource::Imu, tx.clone());
        registry.register_for_input(DataSource::Imu, tx);
        
        assert_eq!(registry.get_registered_transmitters(DataSource::Imu).unwrap().len(), 2);
        assert!(registry.get_registered_transmitters(DataSource::Imu).is_none());
    }
}
