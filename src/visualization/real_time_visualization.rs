use std::{
    collections::VecDeque,
    sync::mpsc::Receiver,
    time::SystemTime
};

use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::prelude::*;
use plotters_piston::{draw_piston_window, PistonBackend};

use crate::{
    config,
    data::{Data, Telemetry},
    visualization::{self, Visualization},
};

#[derive(Debug)]
pub struct RealTimeVisualization {
    visualization: Visualization,
}

impl RealTimeVisualization {
    pub fn new(
        rx_gps: Receiver<Telemetry>,
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,

        rx_inertial: Receiver<Telemetry>,
        rx_groundtruth: Receiver<Telemetry>,
        simulation_start: SystemTime,
    ) -> RealTimeVisualization {
        RealTimeVisualization {
            visualization: Visualization {
                gps_data: VecDeque::from(
                    [Data::new();
                        (config::PLOT_RANGE_WINDOW * config::GPS_FREQ.get() as u128) as usize],
                ),
                avg_data: VecDeque::from(
                    [Data::new();
                        (config::PLOT_RANGE_WINDOW * config::GPS_FREQ.get() as u128) as usize],
                ),
                kalman_data: VecDeque::from(
                    [Data::new();
                        (config::PLOT_RANGE_WINDOW * config::IMU_FREQ.get() as u128) as usize],
                ),
                inertial_data: VecDeque::from(
                    [Data::new();
                        (config::PLOT_RANGE_WINDOW * config::IMU_FREQ.get() as u128) as usize],
                ),
                groundtruth_data: VecDeque::from(
                    [Data::new();
                        (config::PLOT_RANGE_WINDOW * config::GENERATOR_FREQ.get() as u128) as usize],
                ),
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

    pub fn run(receivers: visualization::PlotterReceivers, simulation_start: SystemTime) {
        let mut window: PistonWindow = WindowSettings::new("RustSFD", [1280, 720])
            .samples(4)
            .exit_on_esc(true)
            .build()
            .unwrap();

        window.set_max_fps(config::FPS as u64);

        let mut real_time_visualization = RealTimeVisualization::new(
            receivers.rx_gps,
            receivers.rx_avg,
            receivers.rx_kalman,
            receivers.rx_inertial,
            receivers.rx_groundtruth,
            simulation_start,
        );

        while draw_piston_window(&mut window, |b: PistonBackend<'_, '_>| {
            real_time_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Gps, visualization::VisualizationType::Dynamic);
            real_time_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Avg, visualization::VisualizationType::Dynamic);
            real_time_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Kalman, visualization::VisualizationType::Dynamic);
            real_time_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Inertial, visualization::VisualizationType::Dynamic);
            real_time_visualization
                .visualization
                .get_plot_data(visualization::PlotDataType::Groundtruth, visualization::VisualizationType::Dynamic);
            real_time_visualization.draw(b);

            Ok(())
        })
        .is_some()
        {}
    }

    fn draw(&mut self, b: PistonBackend<'_, '_>) {
        let root: DrawingArea<PistonBackend<'_, '_>, _> = b.into_drawing_area();
        let _ = root.fill(&WHITE);

        let (upper, lower) = root.split_vertically(40);
        upper.titled("RustSDF", ("comic-sans", 30)).unwrap();

        let split_in_3 = lower.split_evenly((3, 1));
        let (_, lower_0) = split_in_3[0].split_vertically(40);
        let (_, lower_1) = split_in_3[1].split_vertically(40);
        let (_, lower_2) = split_in_3[2].split_vertically(40);

        self.update_plot_range();

        self.visualization
            .draw_coordinate(lower_0, visualization::PlotAxis::X);
        self.visualization
            .draw_coordinate(lower_1, visualization::PlotAxis::Y);
        self.visualization
            .draw_coordinate(lower_2, visualization::PlotAxis::Z);
    }

    fn update_plot_range(&mut self) {
        self.visualization.plot_stop = match self.visualization.kalman_data.back() {
            Some(data) => data
                .timestamp
                .duration_since(self.visualization.simulation_start)
                .unwrap()
                .as_millis(),
            None => panic!("Trying to access empty buffer!"),
        };

        self.visualization.plot_start = self
            .visualization
            .kalman_data
            .front()
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
    
    use std::sync::mpsc::{self, Sender};

    use crate::{
        Data,
        visualization,
    };

    fn prepare_test_env() -> (
        RealTimeVisualization,
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

        let real_time_visualization = RealTimeVisualization::new(rx_gps, rx_avg, rx_kalman, rx_inertial, rx_groundtruth, simulation_start);

        (
            real_time_visualization,
            tx_gps,
            tx_avg,
            tx_kalman,
            tx_inertial,
            tx_groundtruth,
        )
    }

    #[test]
    #[should_panic]
    fn test_get_plot_data_wrong_input() {
        let (mut real_time_visualization, _, tx_avg, _, _, _) = prepare_test_env();

        assert!(real_time_visualization.visualization.avg_data.iter().last().unwrap().x == 0.0);

        let _ = tx_avg.send(Telemetry::Acceleration(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.visualization.get_plot_data(visualization::PlotDataType::Avg, visualization::VisualizationType::Dynamic);
        assert!(real_time_visualization.visualization.avg_data.iter().last().unwrap().x == 1.0);
    }

    #[test]
    fn test_get_plot_data_correct_input() {
        let (mut real_time_visualization, tx_gps, tx_avg, tx_kalman, tx_inertial, tx_groundtruth) =
            prepare_test_env();

        assert_eq!(
            real_time_visualization.visualization.gps_data.iter().last().unwrap().x,
            0.0
        );
        assert_eq!(
            real_time_visualization.visualization.avg_data.iter().last().unwrap().x,
            0.0
        );
        assert_eq!(
            real_time_visualization.visualization.kalman_data.iter().last().unwrap().x,
            0.0
        );
        assert_eq!(
            real_time_visualization
                .visualization.inertial_data
                .iter()
                .last()
                .unwrap()
                .x,
            0.0
        );
        assert_eq!(
            real_time_visualization
                .visualization.groundtruth_data
                .iter()
                .last()
                .unwrap()
                .x,
            0.0
        );

        let _ = tx_gps.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.visualization.get_plot_data(visualization::PlotDataType::Gps, visualization::VisualizationType::Dynamic);
        assert_eq!(
            real_time_visualization.visualization.gps_data.iter().last().unwrap().x,
            1.0
        );

        let _ = tx_avg.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.visualization.get_plot_data(visualization::PlotDataType::Avg, visualization::VisualizationType::Dynamic);
        assert_eq!(
            real_time_visualization.visualization.avg_data.iter().last().unwrap().x,
            1.0
        );

        let _ = tx_kalman.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.visualization.get_plot_data(visualization::PlotDataType::Kalman, visualization::VisualizationType::Dynamic);
        assert_eq!(
            real_time_visualization.visualization.kalman_data.iter().last().unwrap().x,
            1.0
        );

        let _ = tx_inertial.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.visualization.get_plot_data(visualization::PlotDataType::Inertial, visualization::VisualizationType::Dynamic);
        assert_eq!(
            real_time_visualization
                .visualization.inertial_data
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
        real_time_visualization.visualization.get_plot_data(visualization::PlotDataType::Groundtruth, visualization::VisualizationType::Dynamic);
        assert_eq!(
            real_time_visualization
                .visualization.groundtruth_data
                .iter()
                .last()
                .unwrap()
                .x,
            1.0
        );
    }
}
