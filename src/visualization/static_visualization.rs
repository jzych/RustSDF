use crate::data::{Data, Telemetry};
use plotters::coord::types::RangedCoordf64;
use plotters::coord::Shift;
use plotters::prelude::*;
use std::collections::VecDeque;
use std::thread;
use std::time::SystemTime;
use std::{sync::mpsc::Receiver, thread::JoinHandle};

use crate::config;
use crate::visualization;

pub struct StaticVisualization {
    gps_data: VecDeque<Data>,
    avg_data: VecDeque<Data>,
    kalman_data: VecDeque<Data>,
    inertial_data: VecDeque<Data>,
    groundtruth_data: VecDeque<Data>,
    rx_gps: Receiver<Telemetry>,
    rx_avg: Receiver<Telemetry>,
    rx_kalman: Receiver<Telemetry>,
    rx_inertial: Receiver<Telemetry>,
    rx_groundtruth: Receiver<Telemetry>,
    plot_start: f64,
    plot_stop: f64,
    simulation_start: SystemTime,
}

impl StaticVisualization {
    fn new(
        rx_gps: Receiver<Telemetry>,
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,
        rx_inertial: Receiver<Telemetry>,
        rx_groundtruth: Receiver<Telemetry>,
        simulation_start: SystemTime,
    ) -> StaticVisualization {
        StaticVisualization {
            gps_data: VecDeque::new(),
            avg_data: VecDeque::new(),
            kalman_data: VecDeque::new(),
            inertial_data: VecDeque::new(),
            groundtruth_data: VecDeque::new(),
            rx_gps,
            rx_avg,
            rx_kalman,
            rx_inertial,
            rx_groundtruth,
            plot_start: 0.0,
            plot_stop: 0.0,
            simulation_start,
        }
    }

    pub fn run(
        receivers: visualization::PlotterReceivers,
        simulation_start: SystemTime,
    ) -> JoinHandle<()> {
        let mut static_visualization = StaticVisualization::new(
            receivers.rx_gps,
            receivers.rx_avg,
            receivers.rx_kalman,
            receivers.rx_inertial,
            receivers.rx_groundtruth,
            simulation_start,
        );

        let handle = thread::spawn(move || {
            static_visualization.get_plot_data(visualization::PlotDataType::Gps);
            static_visualization.get_plot_data(visualization::PlotDataType::Avg);
            static_visualization.get_plot_data(visualization::PlotDataType::Kalman);
            static_visualization.get_plot_data(visualization::PlotDataType::Inertial);
            static_visualization.get_plot_data(visualization::PlotDataType::Groundtruth);
            static_visualization.update_plot_time_range();
            static_visualization.draw();
            println!("Static Visualization removed");
        });
        handle
    }

    fn get_plot_data(&mut self, plot_data_type: visualization::PlotDataType) {
        let (rx, rx_data) = match plot_data_type {
            visualization::PlotDataType::Gps => (&self.rx_gps, &mut self.gps_data),
            visualization::PlotDataType::Avg => (&self.rx_avg, &mut self.avg_data),
            visualization::PlotDataType::Kalman => (&self.rx_kalman, &mut self.kalman_data),
            visualization::PlotDataType::Inertial => (&self.rx_inertial, &mut self.inertial_data),
            visualization::PlotDataType::Groundtruth => {
                (&self.rx_groundtruth, &mut self.groundtruth_data)
            }
        };

        while let Ok(data) = rx.recv() {
            match data {
                Telemetry::Position(d) => {
                    rx_data.push_back(d);
                }
                Telemetry::Acceleration(_) => {
                    panic!("Acceleration should not be passed as an input!");
                }
            }
        }
    }

    fn draw(&mut self) {
        let root =
            BitMapBackend::new("output/plot_gps_avg_kalman.png", (2000, 1000)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let (upper, lower) = root.split_vertically(40);
        upper.titled("RustSDF", ("comic-sans", 30)).unwrap();

        let split_in_3 = lower.split_evenly((3, 1));
        let (_, lower_0) = split_in_3[0].split_vertically(40);
        let (_, lower_1) = split_in_3[1].split_vertically(40);
        let (_, lower_2) = split_in_3[2].split_vertically(40);

        self.create_plot(lower_0, visualization::PlotAxis::X);
        self.create_plot(lower_1, visualization::PlotAxis::Y);
        self.create_plot(lower_2, visualization::PlotAxis::Z);
    }

    fn create_plot(
        &mut self,
        root: DrawingArea<BitMapBackend<'_>, Shift>,
        coord: visualization::PlotAxis,
    ) {
        let mut chart = ChartBuilder::on(&root)
            .x_label_area_size(60)
            .y_label_area_size(60)
            .right_y_label_area_size(60)
            .margin_bottom(30)
            .build_cartesian_2d(self.plot_start..self.plot_stop, -100f64..150f64)
            .unwrap();

        chart
            .configure_mesh()
            .x_desc("time")
            .y_desc(coord.to_string())
            .draw()
            .unwrap();

        self.chart_data(
            &self.groundtruth_data,
            "Groundtruth",
            config::GROUNDTRUTH_PLOT_COLOR,
            &mut chart,
            coord,
        );
        self.chart_data(
            &self.gps_data,
            "GPS with noise",
            config::GPS_PLOT_COLOR,
            &mut chart,
            coord,
        );
        self.chart_data(
            &self.avg_data,
            "Moving GPS average",
            config::AVERAGE_PLOT_COLOR,
            &mut chart,
            coord,
        );
        self.chart_data(
            &self.inertial_data,
            "Inertial navigator",
            config::INERTIAL_PLOT_COLOR,
            &mut chart,
            coord,
        );
        self.chart_data(
            &self.kalman_data,
            "Kalman filter",
            config::KALMAN_PLOT_COLOR,
            &mut chart,
            coord,
        );

        chart
            .configure_series_labels()
            .border_style(BLACK)
            .background_style(WHITE.mix(100.0))
            .position(SeriesLabelPosition::LowerRight)
            .draw()
            .unwrap()
    }

    fn update_plot_time_range(&mut self) {
        self.plot_start = self
            .gps_data
            .front()
            .unwrap()
            .timestamp
            .duration_since(self.simulation_start)
            .unwrap()
            .as_secs_f64();

        self.plot_stop = self
            .gps_data
            .back()
            .unwrap()
            .timestamp
            .duration_since(self.simulation_start)
            .unwrap()
            .as_secs_f64();
    }

    fn chart_data(
        &self,
        data: &VecDeque<Data>,
        label: &str,
        color: RGBColor,
        chart: &mut ChartContext<
            '_,
            BitMapBackend<'_>,
            Cartesian2d<RangedCoordf64, RangedCoordf64>,
        >,
        coord: visualization::PlotAxis,
    ) {
        chart
            .draw_series(LineSeries::new(
                data.iter().map(|p| {
                    (
                        p.timestamp
                            .duration_since(self.simulation_start)
                            .unwrap()
                            .as_secs_f64(),
                        match coord {
                            visualization::PlotAxis::X => p.x,
                            visualization::PlotAxis::Y => p.y,
                            visualization::PlotAxis::Z => p.z,
                        },
                    )
                }),
                color,
            ))
            .unwrap()
            .label(label)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], color));
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     use std::path::Path;
//     use std::time::SystemTime;

//     #[test]
//     fn test_select_xyz() {
//         let data = Data {
//             x: 234.5,
//             y: 555.1,
//             z: 33.3,
//             timestamp: SystemTime::now(),
//         };

//         approx::assert_abs_diff_eq!(select_xyz("x", data), 234.5);
//         approx::assert_abs_diff_eq!(select_xyz("y", data), 555.1);
//         approx::assert_abs_diff_eq!(select_xyz("z", data), 33.3);
//     }

//     #[test]
//     #[should_panic]
//     fn test_select_xyz_wrong_input() {
//         let data = Data {
//             x: 234.5,
//             y: 555.1,
//             z: 33.3,
//             timestamp: SystemTime::now(),
//         };

//         approx::assert_abs_diff_eq!(select_xyz("dariajestsuper", data), 33.3);
//     }

//     #[test]
//     fn test_plot_file_generated() {
//         let simulation_time = SystemTime::now();

//         let avg_data = vec![Data::new()];
//         let kalman_data = vec![Data::new()];
//         let gps_data = vec![Data::new()];
//         let inertial_data = vec![Data::new()];
//         let groundtruth_data = vec![Data::new()];

//         draw(
//             avg_data,
//             kalman_data,
//             gps_data,
//             inertial_data,
//             groundtruth_data,
//             simulation_time,
//         );

//         let path = Path::new("output/plot_gps_avg_kalman.png");
//         assert!(path.exists());
//     }
// }
