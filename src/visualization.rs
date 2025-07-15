use crate::{
    data::{Data, Telemetry},
    imu,
};
use plotters::{data, prelude::*};
use rand::Rng;
use std::{thread, time::{SystemTime, UNIX_EPOCH}};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex,
    },
    thread::{JoinHandle},
};

use crate::{
    communication_registry::{CommunicationRegistry, DataSource},
};

#[allow(unused)]
pub struct Visualization;

impl Visualization {
    pub fn consume_data_for_visualization(source: DataSource, consumer_registry: &mut CommunicationRegistry) -> JoinHandle<()> {
        let (tx, input_rx) = mpsc::channel();
        consumer_registry.register_for_input(source, tx);
        let mut buffer = Vec::new();

        let handle = thread::spawn(move || {
        for data in input_rx {
            match data {
                Telemetry::Acceleration(d) => {
                    buffer.push(d);

                }
                Telemetry::Position(d) => {
                    buffer.push(d);
                }
            }
        }
        println!("Channel has been closed, exiting the thread.");
    });
    handle
    }
}

fn draw() {
    let sensor_name = "TODO";
    let complete_plot_name = format!("output/{sensor_name}.png"); //TODO:add error handling if dir is not available?

    let root = BitMapBackend::new(&complete_plot_name, (2000, 1000)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let (upper, lower) = root.split_vertically(40);
    upper.titled("TODO", ("comic-sans", 30)).unwrap();

    let split_in_3 = lower.split_evenly((3, 1));
    let (upper_0, lower_0) = split_in_3[0].split_vertically(40);
    let (upper_1, lower_1) = split_in_3[1].split_vertically(40);
    let (upper_2, lower_2) = split_in_3[2].split_vertically(40);

    let mut chart_lower_0 = ChartBuilder::on(&lower_0)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .build_cartesian_2d(0f64..100f64, 0f64..1f64)
        .unwrap();

    chart_lower_0
        .configure_mesh()
        .x_desc("timestamp")
        .y_desc("x")
        .draw()
        .unwrap();

    chart_lower_0
        .draw_series(LineSeries::new(
            vec![(0.0, 0.0), (5.0, 5.0), (8.0, 7.0)],
            &RED,
        ))
        .unwrap()
        .label("Kalman filter")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED)); //TODO: investigate PathElem::new

    chart_lower_0
        .draw_series(LineSeries::new(
            vec![(0.0, 0.0), (10.0, 5.0), (30.0, 7.0)],
            &BLUE,
        ))
        .unwrap()
        .label("Average filter")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

    chart_lower_0.configure_series_labels()
        .border_style(&BLACK)
        .background_style(&WHITE.mix(0.0))
        .draw()
        .unwrap()
}

#[cfg(test)]
mod tests {
    use crate::visualization::draw;

    #[test]
    fn dummy() {
        draw();
        assert!(true);
    }
}
