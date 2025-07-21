use plotters::coord::Shift;
use plotters::prelude::*;
use std::{sync::mpsc::Receiver, thread::JoinHandle};
use std::{thread, time::UNIX_EPOCH};

use crate::data::{Data, Telemetry};

#[allow(unused)]
pub struct Visualization;

impl Visualization {
    pub fn run(
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,
        rx_gps: Receiver<Telemetry>,
    ) -> JoinHandle<()> {
        let mut avg_data = Vec::new();
        let mut kalman_data = Vec::new();
        let mut gps_data = Vec::new();

        let handle = thread::spawn(move || {
            for data in rx_gps {
                match data {
                    Telemetry::Position(d) => {
                        gps_data.push(d);
                    }
                    Telemetry::Acceleration(_d) => {
                        //avg_data.push(data);
                    }
                }
            }

            for data in rx_avg {
                match data {
                    Telemetry::Position(d) => {
                        avg_data.push(d);
                    }
                    Telemetry::Acceleration(_d) => {
                        //avg_data.push(data);
                    }
                }
            }

            for data in rx_kalman {
                match data {
                    Telemetry::Position(d) => {
                        kalman_data.push(d);
                    }
                    Telemetry::Acceleration(_d) => {
                        //avg_data.push(data);
                    }
                }
            }
            // println!(
            //     "Len avg = {} kalman = {} gps = {}",
            //     avg_data.len(),
            //     kalman_data.len(),
            //     gps_data.len()
            // );
            draw(avg_data, kalman_data, gps_data);
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

fn create_plot(
    root: DrawingArea<BitMapBackend<'_>, Shift>,
    coord_to_plot: &str,
    avg_data: &[Data],
    kalman_data: &[Data],
    gps_data: &[Data],
) {
    let plot_start = gps_data[0]
        .timestamp
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as f64;
    let plot_stop = gps_data
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
        .build_cartesian_2d(plot_start..plot_stop, 0f64..200f64)
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
        .draw_series(LineSeries::new(
            gps_data.iter().map(|p| {
                (
                    p.timestamp.duration_since(UNIX_EPOCH).unwrap().as_millis() as f64,
                    select_xyz(coord_to_plot, *p),
                )
            }),
            &GREEN,
        ))
        .unwrap()
        .label("GPS real data")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], GREEN));

    chart
        .configure_series_labels()
        .border_style(BLACK)
        .background_style(WHITE.mix(0.0))
        .draw()
        .unwrap()
}

fn draw(avg_data: Vec<Data>, kalman_data: Vec<Data>, gps_data: Vec<Data>) {
    let root = BitMapBackend::new("output/plot_gps_avg_kalman.png", (2000, 1000)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let (upper, lower) = root.split_vertically(40);
    upper.titled("RustSDF", ("comic-sans", 30)).unwrap();

    let split_in_3 = lower.split_evenly((3, 1));
    let (_, lower_0) = split_in_3[0].split_vertically(40);
    let (_, lower_1) = split_in_3[1].split_vertically(40);
    let (_, lower_2) = split_in_3[2].split_vertically(40);

    create_plot(lower_0, "x", &avg_data, &kalman_data, &gps_data);
    create_plot(lower_1, "y", &avg_data, &kalman_data, &gps_data);
    create_plot(lower_2, "z", &avg_data, &kalman_data, &gps_data);
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::SystemTime;

    #[test]
    fn test_select_xyz() {
        let data = Data {
            x: 234.5,
            y: 555.1,
            z: 33.3,
            timestamp: SystemTime::now(),
        };

        assert!(select_xyz("x", data) == 234.5);
        assert!(select_xyz("y", data) == 555.1);
        assert!(select_xyz("z", data) == 33.3);

    }
}
