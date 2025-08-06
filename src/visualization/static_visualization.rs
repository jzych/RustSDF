use std::{
    collections::VecDeque, sync::mpsc::Receiver, thread, thread::JoinHandle, time::SystemTime,
};

use plotters::prelude::*;

use crate::{
    data::Telemetry,
    logger::log,
    log_config::GENERAL_LOG,
    visualization::{self, Visualization},
};

#[derive(Debug)]
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

        thread::spawn(move || {
            static_visualization.visualization.get_plot_data(
                visualization::PlotDataType::Gps,
                visualization::VisualizationType::Static,
            );
            static_visualization.visualization.get_plot_data(
                visualization::PlotDataType::Avg,
                visualization::VisualizationType::Static,
            );
            static_visualization.visualization.get_plot_data(
                visualization::PlotDataType::Kalman,
                visualization::VisualizationType::Static,
            );
            static_visualization.visualization.get_plot_data(
                visualization::PlotDataType::Inertial,
                visualization::VisualizationType::Static,
            );
            static_visualization.visualization.get_plot_data(
                visualization::PlotDataType::Groundtruth,
                visualization::VisualizationType::Static,
            );

            static_visualization.update_plot_range();
            static_visualization.draw();
            log(GENERAL_LOG, "Static visualization removed".to_string());
        })
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

#[cfg(test)]
mod tests {
    use super::*;

    use std::{
        path::Path,
        sync::mpsc::{self, Sender},
        thread::sleep,
        time::Duration,
    };

    use crate::{visualization, Data};

    fn prepare_test_env() -> (
        StaticVisualization,
        Sender<Telemetry>,
        Sender<Telemetry>,
        Sender<Telemetry>,
        Sender<Telemetry>,
        Sender<Telemetry>,
    ) {
        let simulation_start = SystemTime::now();
        let (tx_gps, rx_gps) = mpsc::channel();
        let (tx_avg, rx_avg) = mpsc::channel();
        let (tx_kalman, rx_kalman) = mpsc::channel();
        let (tx_inertial, rx_inertial) = mpsc::channel();
        let (tx_groundtruth, rx_groundtruth) = mpsc::channel();

        let static_visualization = StaticVisualization::new(
            rx_gps,
            rx_avg,
            rx_kalman,
            rx_inertial,
            rx_groundtruth,
            simulation_start,
        );

        (
            static_visualization,
            tx_gps,
            tx_avg,
            tx_kalman,
            tx_inertial,
            tx_groundtruth,
        )
    }

    #[test]
    fn test_plot_file_generated() {
        let (mut static_visualization, _, _, _, _, _) = prepare_test_env();
        static_visualization.draw();

        let path = Path::new("output/plot_gps_avg_kalman.png");
        assert!(path.exists());
    }

    #[test]
    #[should_panic]
    fn test_get_plot_data_wrong_input() {
        let (mut static_visualization, _, tx_avg, _, _, _) = prepare_test_env();

        assert!(
            static_visualization
                .visualization
                .avg_data
                .iter()
                .last()
                .unwrap()
                .x
                == 0.0
        );

        let _ = tx_avg.send(Telemetry::Acceleration(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        static_visualization.visualization.get_plot_data(
            visualization::PlotDataType::Avg,
            visualization::VisualizationType::Static,
        );
        assert!(
            static_visualization
                .visualization
                .avg_data
                .iter()
                .last()
                .unwrap()
                .x
                == 1.0
        );
    }

    #[test]
    fn test_get_plot_data_correct_input() {
        let (mut static_visualization, tx_gps, tx_avg, tx_kalman, tx_inertial, tx_groundtruth) =
            prepare_test_env();

        assert_eq!(static_visualization.visualization.gps_data.len(), 0);
        assert_eq!(static_visualization.visualization.avg_data.len(), 0);
        assert_eq!(static_visualization.visualization.kalman_data.len(), 0);
        assert_eq!(static_visualization.visualization.inertial_data.len(), 0);
        assert_eq!(static_visualization.visualization.groundtruth_data.len(), 0);

        let _ = tx_gps.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        drop(tx_gps);
        static_visualization.visualization.get_plot_data(
            visualization::PlotDataType::Gps,
            visualization::VisualizationType::Static,
        );
        assert_eq!(
            static_visualization
                .visualization
                .gps_data
                .iter()
                .last()
                .unwrap()
                .x,
            1.0
        );

        let _ = tx_avg.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        drop(tx_avg);
        static_visualization.visualization.get_plot_data(
            visualization::PlotDataType::Avg,
            visualization::VisualizationType::Static,
        );
        assert_eq!(
            static_visualization
                .visualization
                .avg_data
                .iter()
                .last()
                .unwrap()
                .x,
            1.0
        );

        let _ = tx_kalman.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        drop(tx_kalman);
        static_visualization.visualization.get_plot_data(
            visualization::PlotDataType::Kalman,
            visualization::VisualizationType::Static,
        );
        assert_eq!(
            static_visualization
                .visualization
                .kalman_data
                .iter()
                .last()
                .unwrap()
                .x,
            1.0
        );

        let _ = tx_inertial.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        drop(tx_inertial);
        static_visualization.visualization.get_plot_data(
            visualization::PlotDataType::Inertial,
            visualization::VisualizationType::Static,
        );
        assert_eq!(
            static_visualization
                .visualization
                .inertial_data
                .iter()
                .last()
                .unwrap()
                .x,
            1.0
        );

        let _ = tx_groundtruth.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        drop(tx_groundtruth);
        static_visualization.visualization.get_plot_data(
            visualization::PlotDataType::Groundtruth,
            visualization::VisualizationType::Static,
        );
        assert_eq!(
            static_visualization
                .visualization
                .groundtruth_data
                .iter()
                .last()
                .unwrap()
                .x,
            1.0
        );
    }

    #[test]
    fn test_update_plot_range_start_value() {
        let (mut real_time_visualization, _, _, _, _, _) = prepare_test_env();

        sleep(Duration::from_millis(100));
        let gps_time = SystemTime::now();

        real_time_visualization
            .visualization
            .gps_data
            .push_front(Data {
                x: 77.7,
                y: 77.7,
                z: 77.7,
                timestamp: gps_time,
            });

        real_time_visualization
            .visualization
            .gps_data
            .push_back(Data {
                x: 33.3,
                y: 33.3,
                z: 33.3,
                timestamp: SystemTime::now(),
            });

        assert_eq!(
            real_time_visualization
                .visualization
                .gps_data
                .front()
                .unwrap()
                .x,
            77.7
        );
        assert_eq!(
            real_time_visualization
                .visualization
                .gps_data
                .back()
                .unwrap()
                .x,
            33.3
        );

        assert_ne!(
            real_time_visualization.visualization.plot_start,
            gps_time
                .duration_since(real_time_visualization.visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
        real_time_visualization.update_plot_range();
        assert_eq!(
            real_time_visualization.visualization.plot_start,
            gps_time
                .duration_since(real_time_visualization.visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
    }

    #[test]
    fn test_update_plot_range_stop_value() {
        let (mut real_time_visualization, _, _, _, _, _) = prepare_test_env();

        sleep(Duration::from_millis(100));
        let gps_time = SystemTime::now();

        real_time_visualization
            .visualization
            .gps_data
            .push_back(Data {
                x: 77.7,
                y: 77.7,
                z: 77.7,
                timestamp: gps_time,
            });

        real_time_visualization
            .visualization
            .gps_data
            .push_front(Data {
                x: 33.3,
                y: 33.3,
                z: 33.3,
                timestamp: SystemTime::now(),
            });

        assert_eq!(
            real_time_visualization
                .visualization
                .gps_data
                .front()
                .unwrap()
                .x,
            33.3
        );
        assert_eq!(
            real_time_visualization
                .visualization
                .gps_data
                .back()
                .unwrap()
                .x,
            77.7
        );

        assert_ne!(
            real_time_visualization.visualization.plot_stop,
            gps_time
                .duration_since(real_time_visualization.visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
        real_time_visualization.update_plot_range();
        assert_eq!(
            real_time_visualization.visualization.plot_stop,
            gps_time
                .duration_since(real_time_visualization.visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
    }
}
