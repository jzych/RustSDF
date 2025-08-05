use crate::data::Telemetry;
use plotters::prelude::*;
use std::collections::VecDeque;
use std::thread;
use std::time::SystemTime;
use std::{sync::mpsc::Receiver, thread::JoinHandle};

use crate::visualization::{self, Visualization};

pub struct StaticVisualization {
    visualization: Visualization,
}

impl StaticVisualization {
    pub fn new(
        rx_gps: Receiver<Telemetry>,
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,
        rx_inertial: Receiver<Telemetry>,
        rx_groundtruth: Receiver<Telemetry>,
        simulation_start: SystemTime,
    ) -> StaticVisualization {
        StaticVisualization {
            visualization: Visualization {
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
                plot_start: SystemTime::now()
                    .duration_since(simulation_start)
                    .unwrap()
                    .as_millis(),
                plot_stop: SystemTime::now()
                    .duration_since(simulation_start)
                    .unwrap()
                    .as_millis(),
                simulation_start,
            },
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
            static_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Gps, visualization::VisualizationType::Static);
            static_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Avg, visualization::VisualizationType::Static);
            static_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Kalman, visualization::VisualizationType::Static);
            static_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Inertial, visualization::VisualizationType::Static);
            static_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Groundtruth, visualization::VisualizationType::Static);

            static_visualization.update_plot_range();
            static_visualization.draw();
            println!("Static Visualization removed");
        });
        handle
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

        self.visualization
            .draw_coordinate(lower_0, visualization::PlotAxis::X);
        self.visualization
            .draw_coordinate(lower_1, visualization::PlotAxis::Y);
        self.visualization
            .draw_coordinate(lower_2, visualization::PlotAxis::Z);
    }

    fn update_plot_range(&mut self) {
        self.visualization.plot_start = self
            .visualization
            .gps_data
            .front()
            .unwrap()
            .timestamp
            .duration_since(self.visualization.simulation_start)
            .unwrap()
            .as_millis();

        self.visualization.plot_stop = self
            .visualization
            .gps_data
            .back()
            .unwrap()
            .timestamp
            .duration_since(self.visualization.simulation_start)
            .unwrap()
            .as_millis();
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
