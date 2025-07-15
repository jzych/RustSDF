use std::{
    collections::VecDeque,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{Receiver, Sender}, 
        Arc,
    },
    thread::{self, JoinHandle},
    time::SystemTime,
};

use crate::data::{Data, Telemetry};

#[allow(unused)]
pub struct Average;

impl Average {
    pub fn run(mut tx: Vec<Sender<Telemetry>>, rx: Receiver<Telemetry>, shutdown: Arc<AtomicBool>) -> JoinHandle<()> {
        thread::spawn(move || {
            let mut buffer = VecDeque::new();

            while !shutdown.load(Ordering::SeqCst) {
                if let Ok(Telemetry::Position(new_data)) = rx.recv() {
                    Average::handle_data_buffer(&mut buffer, new_data);
                    let avg_data = Average::calculate_average(&buffer);
                    tx.retain(|tx| tx.send(Telemetry::Position(avg_data)).is_ok());
                }
                
                if tx.is_empty() {
                    break;
                }
            }
            println!("Average filter removed");
        })
    }

    pub fn handle_data_buffer(buffer: &mut VecDeque<Data>, new_data: Data) {
        if buffer.len() < 10 {
            buffer.push_back(new_data);
        } else {
            buffer.pop_front();
            buffer.push_back(new_data);
        }
    }

    pub fn calculate_average(buffer: &VecDeque<Data>) -> Data {
        let buffer_iter = buffer.iter();
        let count = buffer.len();
        let mut sum_x: f64 = 0.0;
        let mut sum_y: f64 = 0.0;
        let mut sum_z: f64 = 0.0;

        for elem in buffer_iter {
            sum_x += elem.x;
            sum_y += elem.y;
            sum_z += elem.z;
        }

        Data {
            x: sum_x / count as f64,
            y: sum_y / count as f64,
            z: sum_z / count as f64,
            timestamp: SystemTime::now(),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use rand::Rng;
    use std::collections::VecDeque;
    use std::time::SystemTime;

    #[test]
    fn test_calculate_average() {
        let mut buffer: VecDeque<Data> = VecDeque::new();
        buffer.push_back(Data {
            x: 2.2,
            y: 4.5,
            z: 7.8,
            timestamp: SystemTime::now(),
        });
        buffer.push_back(Data {
            x: 44.6,
            y: 88.2,
            z: 987.1,
            timestamp: SystemTime::now(),
        });
        buffer.push_back(Data {
            x: 234.5,
            y: 555.1,
            z: 33.3,
            timestamp: SystemTime::now(),
        });

        let local_avg_x: f64 = (buffer[0].x + buffer[1].x + buffer[2].x) / 3.0;
        let local_avg_y: f64 = (buffer[0].y + buffer[1].y + buffer[2].y) / 3.0;
        let local_avg_z: f64 = (buffer[0].z + buffer[1].z + buffer[2].z) / 3.0;

        let calculate_average_output: Data = Average::calculate_average(&buffer);

        assert!(calculate_average_output.x == local_avg_x);
        assert!(calculate_average_output.y == local_avg_y);
        assert!(calculate_average_output.z == local_avg_z);
    }

    #[test]
    fn test_handle_data_buffer() {
        let mut buffer = VecDeque::new();
        let data = Data {
            x: 234.5,
            y: 555.1,
            z: 33.3,
            timestamp: SystemTime::now(),
        };

        gen_vectors(2, &mut buffer);
        Average::handle_data_buffer(&mut buffer, data);
        assert!(buffer.len() == 3);
        assert!(buffer[2].x == data.x);

        buffer.clear();
        assert!(buffer.is_empty());

        gen_vectors(10, &mut buffer);
        Average::handle_data_buffer(&mut buffer, data);
        assert!(buffer.len() == 10);
        assert!(buffer[9].x == data.x);
    }

    fn gen_vectors(vec_len: u8, buffer: &mut VecDeque<Data>) {
        let mut rng = rand::rng();
        for _ in 0..vec_len {
            buffer.push_back(Data {
                x: rng.random_range(0.0..=100.0),
                y: rng.random_range(0.0..=100.0),
                z: rng.random_range(0.0..=100.0),
                timestamp: SystemTime::now(),
            });
        }
    }
}
