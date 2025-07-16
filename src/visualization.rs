use crate::{
    data::{Data, Telemetry},
    imu,
};
use plotters::{data, prelude::*};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{Sender, Receiver},
    },
    thread::JoinHandle,
};
use std::{
    thread,
    time::{SystemTime, UNIX_EPOCH},
};
use plotters::coord::Shift;


#[allow(unused)]
pub struct Visualization;

impl Visualization {
    pub fn run(rx_avg: Receiver<Telemetry>, rx_kalman: Receiver<Telemetry>) -> JoinHandle<()> {
        let mut avg_data = Vec::new();
        let mut kalman_data = Vec::new();

        let handle = thread::spawn(move || {
            for data in rx_avg {
                match data {
                    Telemetry::Position(d) => {
                        avg_data.push(d);
                    }
                    Telemetry::Acceleration(d) => {
                        //avg_data.push(data);
                    }
                }
            }

            for data in rx_kalman {
                match data {
                    Telemetry::Position(d) => {
                        kalman_data.push(d);
                    }
                    Telemetry::Acceleration(d) => {
                        //avg_data.push(data);
                    }
                }
            }

            draw(avg_data, kalman_data);
            println!("Visualization removed");
        });
        handle
    }
}

fn select_xyz(coord_to_plot: &str, p: Data) -> f64 {
    if coord_to_plot == "x" {
        p.x
    } else if coord_to_plot == "y" {
        p.y
    } else {
        p.z
    }
}

fn create_plot(root: DrawingArea<BitMapBackend<'_>, Shift>, coord_to_plot: &str, avg_data: &Vec<Data>, kalman_data: &Vec<Data>) {

    let plot_start = avg_data[0]
        .timestamp
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as f64;
    let plot_stop = avg_data
        .last()
        .unwrap()
        .timestamp
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as f64;

    let mut chart = ChartBuilder::on(&root)
        .x_label_area_size(60)
        .y_label_area_size(60)
        .right_y_label_area_size(60)
        .margin_bottom(30)
        .build_cartesian_2d(plot_start..plot_stop, 0f64..100f64)
        .unwrap();

    chart
        .configure_mesh()
        .x_desc("time")
        .y_desc(coord_to_plot)
        .draw()
        .unwrap();

    chart
        .draw_series(LineSeries::new(
            kalman_data.iter().map(|p| {
                (
                    p.timestamp.duration_since(UNIX_EPOCH).unwrap().as_millis() as f64,
                    select_xyz(coord_to_plot, *p),
                )
            }),
            &RED,
        ))
        .unwrap()
        .label("Kalman filter")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED)); //TODO: investigate PathElem::new

    chart
        .draw_series(LineSeries::new(
            avg_data.iter().map(|p| {
                (
                    p.timestamp.duration_since(UNIX_EPOCH).unwrap().as_millis() as f64,
                    select_xyz(coord_to_plot, *p),
                )
            }),
            &BLUE,
        ))
        .unwrap()
        .label("Moving average filter")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

    chart
        .configure_series_labels()
        .border_style(BLACK)
        .background_style(WHITE.mix(0.0))
        .draw()
        .unwrap()
}

fn draw(avg_data: Vec<Data>, kalman_data: Vec<Data>) {
    let sensor_name = "TODO";
    let complete_plot_name = format!("output/{sensor_name}.png"); //TODO:add error handling if dir is not available?

    let root = BitMapBackend::new(&complete_plot_name, (2000, 1000)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let (upper, lower) = root.split_vertically(40);
    upper.titled("TODO", ("comic-sans", 30)).unwrap();

    let split_in_3 = lower.split_evenly((3, 1));
    let (_, lower_0) = split_in_3[0].split_vertically(40);
    let (_, lower_1) = split_in_3[1].split_vertically(40);
    let (_, lower_2) = split_in_3[2].split_vertically(40);

    create_plot(lower_0, "x", &avg_data, &kalman_data);
    create_plot(lower_1, "y", &avg_data, &kalman_data);
    create_plot(lower_2, "z", &avg_data, &kalman_data);
}

#[cfg(test)]
mod tests {
    use crate::visualization::draw;

    #[test]
    fn dummy() {
        //draw();
        assert!(true);
    }
}
