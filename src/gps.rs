use crate::data::{Data, Telemetry};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::Sender,
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

const FREQUENCY: f64 = 5.0;

#[allow(unused)]
pub struct Gps;

#[allow(unused)]
impl Gps {
    pub fn run(
        trajectory_generator: Arc<Mutex<Data>>,
        mut tx: Vec<Sender<Telemetry>>,
        shutdown: Arc<AtomicBool>,
    ) -> JoinHandle<()> {
        std::thread::spawn(move || {
            while !shutdown.load(Ordering::SeqCst) {
                let current_position = *trajectory_generator.lock().unwrap();

                tx.retain(|tx| tx.send(Telemetry::Position(current_position)).is_ok());
                if tx.is_empty() {
                    break;
                }
                thread::sleep(Duration::from_secs_f64(1.0 / FREQUENCY));
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
        let gps = Gps::run(
            Arc::clone(&trajectory_generator),
            vec![tx],
            Arc::clone(&shutdown_trigger),
        );
        drop(rx);
        gps.join().unwrap();
    }

    #[test]
    fn gps_produces_positions() {
        let trajectory_generator = Arc::new(Mutex::new(Data::new()));
        let (tx, rx) = mpsc::channel();
        let shutdown_trigger = Arc::new(AtomicBool::new(false));
        let imu = Gps::run(
            Arc::clone(&trajectory_generator),
            vec![tx],
            Arc::clone(&shutdown_trigger),
        );
        std::thread::sleep(Duration::from_secs(1));
        shutdown_trigger.store(true, Ordering::SeqCst);
        imu.join().unwrap();
        let received: Vec<_> = rx.try_iter().collect();
        assert!(!received.is_empty());
    }
}
