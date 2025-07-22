use crate::{
    data::{Data, Telemetry},
    utils::get_cycle_duration,
};
use std::{
    num::NonZeroU32,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::Sender,
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
};

pub struct Gps;

impl Gps {
    pub fn run(
        trajectory_generator: Arc<Mutex<Data>>,
        mut tx: Vec<Sender<Telemetry>>,
        shutdown: Arc<AtomicBool>,
        frequency: NonZeroU32,
    ) -> JoinHandle<()> {
        std::thread::spawn(move || {
            while !shutdown.load(Ordering::SeqCst) {
                let current_position = *trajectory_generator.lock().unwrap();

                tx.retain(|tx| tx.send(Telemetry::Position(current_position)).is_ok());
                if tx.is_empty() {
                    break;
                }
                thread::sleep(get_cycle_duration(frequency));
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    #[test]
    fn given_rx_goes_out_of_scope_gps_shuts_down() {
        let trajectory_generator = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let arbitrary_frequency = NonZeroU32::new(5).unwrap();
        let gps = Gps::run(
            trajectory_generator,
            vec![tx],
            shutdown_trigger,
            arbitrary_frequency,
        );
        drop(rx);
        gps.join().unwrap();
    }

    #[test]
    fn gps_produces_positions() {
        let trajectory_generator = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let arbitrary_frequency = NonZeroU32::new(5).unwrap();
        let gps = Gps::run(
            trajectory_generator,
            vec![tx],
            Arc::clone(&shutdown_trigger),
            arbitrary_frequency,
        );
        let two_cycles = 2 * get_cycle_duration(arbitrary_frequency);
        std::thread::sleep(two_cycles);
        shutdown_trigger.store(true, Ordering::SeqCst);
        gps.join().unwrap();
        let received: Vec<_> = rx.try_iter().collect();
        assert!(!received.is_empty());
    }
}
