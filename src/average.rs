use std::collections::VecDeque;
use std::sync::mpsc;
use std::thread;
use std::time::SystemTime;

use crate::data::Data;

//TODO: read only IMU data
#[warn(while_true)]
// fn handle_average_thread(rx_input: mpsc::Receiver<Data>) {
//     thread::spawn(move || {
//         let mut buffer = VecDeque::new();

//         loop {
//             let new_data = match rx_input.recv() {
//                 Ok(data) => data,
//                 Err(_) => break,
//             };

//             handle_data_buffer(&mut buffer, new_data);
//             calculate_average(&buffer);
//             let recv_handle = send_calculated_average(new_data);
//         }
//     });
// }

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

pub fn send_calculated_average(data: Data) -> mpsc::Receiver<Data> {
    println!("Average filter result: x = {}, y = {}, z = {}", data.x, data.y, data.z);
    let (sender, receiver) = mpsc::channel();
    sender.send(data).unwrap();
    receiver
}

#[cfg(test)]
mod test {
    use rand::Rng;
    use std::collections::VecDeque;
    use std::time::SystemTime;

    use crate::{average::calculate_average, data::{self, Data}};

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

        let calculate_average_output: Data = calculate_average(&buffer);

        assert!(calculate_average_output.x == local_avg_x);
        assert!(calculate_average_output.y == local_avg_y);
        assert!(calculate_average_output.z == local_avg_z);
    }
 
}
