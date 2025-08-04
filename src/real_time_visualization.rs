use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::coord::types::{RangedCoordf64, RangedCoordu128};
use plotters::coord::Shift;
use plotters::prelude::*;
use plotters_piston::{draw_piston_window, PistonBackend};

use crate::config;
use crate::data::{Data, Telemetry};

use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::time::SystemTime;

pub struct PlotterReceivers {
    pub rx_gps: Receiver<Telemetry>,
    pub rx_avg: Receiver<Telemetry>,
    pub rx_kalman: Receiver<Telemetry>,
    pub rx_inertial: Receiver<Telemetry>,
    pub rx_groundtruth: Receiver<Telemetry>,
}

enum PlotDataType {
    Gps,
    Avg,
    Kalman,
    Inertial,
    Groundtruth,
}

#[derive(Debug, Clone, Copy)]
enum PlotAxis {
    X,
    Y,
    Z,
}

impl std::fmt::Display for PlotAxis {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

pub struct RealTimeVisualization {
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
    plot_start: u128,
    plot_stop: u128,
    simulation_start: SystemTime,
}

impl RealTimeVisualization {
    fn new(
        rx_gps: Receiver<Telemetry>,
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,

        rx_inertial: Receiver<Telemetry>,
        rx_groundtruth: Receiver<Telemetry>,
        simulation_start: SystemTime,
    ) -> RealTimeVisualization {
        RealTimeVisualization {
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
        }
    }

    pub fn run(receivers: PlotterReceivers, simulation_start: SystemTime) {
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
            real_time_visualization.get_plot_data(PlotDataType::Gps);
            real_time_visualization.get_plot_data(PlotDataType::Avg);
            real_time_visualization.get_plot_data(PlotDataType::Kalman);
            real_time_visualization.get_plot_data(PlotDataType::Inertial);
            real_time_visualization.get_plot_data(PlotDataType::Groundtruth);
            real_time_visualization.draw(b);

            Ok(())
        })
        .is_some()
        {}
    }

    fn get_plot_data(&mut self, plot_data_type: PlotDataType) {
        let (rx, rx_data) = match plot_data_type {
            PlotDataType::Gps => (&self.rx_gps, &mut self.gps_data),
            PlotDataType::Avg => (&self.rx_avg, &mut self.avg_data),
            PlotDataType::Kalman => (&self.rx_kalman, &mut self.kalman_data),
            PlotDataType::Inertial => (&self.rx_inertial, &mut self.inertial_data),
            PlotDataType::Groundtruth => (&self.rx_groundtruth, &mut self.groundtruth_data),
        };

        while let Ok(data) = rx.try_recv() {
            match data {
                Telemetry::Position(d) => {
                    rx_data.pop_front();
                    rx_data.push_back(d);
                }
                Telemetry::Acceleration(_) => {
                    panic!("Acceleration should not be passed as an input!");
                }
            }
        }
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

        self.draw_coordinate(lower_0, PlotAxis::X);
        self.draw_coordinate(lower_1, PlotAxis::Y);
        self.draw_coordinate(lower_2, PlotAxis::Z);
    }

    fn draw_coordinate(
        &mut self,
        root: DrawingArea<PistonBackend<'_, '_>, Shift>,
        coord: PlotAxis,
    ) {
        let mut chart = ChartBuilder::on(&root)
            .x_label_area_size(40)
            .y_label_area_size(50)
            .right_y_label_area_size(60)
            .margin_bottom(30)
            .build_cartesian_2d(
                self.plot_start..self.plot_stop,
                config::PLOT_RANGE_Y_AXIS_MIN..config::PLOT_RANGE_Y_AXIS_MAX,
            )
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
            .unwrap();
    }

    fn update_plot_range(&mut self) {
        self.plot_stop = match self.kalman_data.back() {
            Some(data) => data
                .timestamp
                .duration_since(self.simulation_start)
                .unwrap()
                .as_millis(),
            None => panic!("Trying to access empty buffer!"),
        };

        self.plot_start = self
            .kalman_data
            .front()
            .unwrap()
            .timestamp
            .duration_since(self.simulation_start)
            .unwrap()
            .as_millis();
    }

    fn chart_data(
        &self,
        data: &VecDeque<Data>,
        label: &str,
        color: RGBColor,
        chart: &mut ChartContext<
            '_,
            PistonBackend<'_, '_>,
            Cartesian2d<RangedCoordu128, RangedCoordf64>,
        >,
        coord: PlotAxis,
    ) {
        chart
            .draw_series(LineSeries::new(
                data.iter().map(|p| {
                    (
                        p.timestamp
                            .duration_since(self.simulation_start)
                            .unwrap()
                            .as_millis(),
                        match coord {
                            PlotAxis::X => p.x,
                            PlotAxis::Y => p.y,
                            PlotAxis::Z => p.z,
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

#[cfg(test)]
mod tests {
    use super::*;

    use std::{
        sync::mpsc::{self, Sender},
        thread::sleep,
        time::Duration,
    };

    #[allow(clippy::type_complexity)]
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

        let real_time_visualization = RealTimeVisualization::new(
            rx_gps,
            rx_avg,
            rx_kalman,
            rx_inertial,
            rx_groundtruth,
            simulation_start,
        );

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

        assert!(real_time_visualization.avg_data.iter().last().unwrap().x == 0.0);

        let _ = tx_avg.send(Telemetry::Acceleration(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.get_plot_data(PlotDataType::Avg);
        assert!(real_time_visualization.avg_data.iter().last().unwrap().x == 1.0);
    }

    #[test]
    fn test_get_plot_data_correct_input() {
        let (mut real_time_visualization, tx_gps, tx_avg, tx_kalman, tx_inertial, tx_groundtruth) =
            prepare_test_env();

        assert_eq!(
            real_time_visualization.gps_data.iter().last().unwrap().x,
            0.0
        );
        assert_eq!(
            real_time_visualization.avg_data.iter().last().unwrap().x,
            0.0
        );
        assert_eq!(
            real_time_visualization.kalman_data.iter().last().unwrap().x,
            0.0
        );
        assert_eq!(
            real_time_visualization
                .inertial_data
                .iter()
                .last()
                .unwrap()
                .x,
            0.0
        );
        assert_eq!(
            real_time_visualization
                .groundtruth_data
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
        real_time_visualization.get_plot_data(PlotDataType::Gps);
        assert_eq!(
            real_time_visualization.gps_data.iter().last().unwrap().x,
            1.0
        );

        let _ = tx_avg.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.get_plot_data(PlotDataType::Avg);
        assert_eq!(
            real_time_visualization.avg_data.iter().last().unwrap().x,
            1.0
        );

        let _ = tx_kalman.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.get_plot_data(PlotDataType::Kalman);
        assert_eq!(
            real_time_visualization.kalman_data.iter().last().unwrap().x,
            1.0
        );

        let _ = tx_inertial.send(Telemetry::Position(Data {
            x: 1.0,
            y: 1.0,
            z: 1.0,
            timestamp: SystemTime::now(),
        }));
        real_time_visualization.get_plot_data(PlotDataType::Inertial);
        assert_eq!(
            real_time_visualization
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
        real_time_visualization.get_plot_data(PlotDataType::Groundtruth);
        assert_eq!(
            real_time_visualization
                .groundtruth_data
                .iter()
                .last()
                .unwrap()
                .x,
            1.0
        );
    }

    #[test]
    fn test_update_plot_range_stop_value() {
        let (mut real_time_visualization, _, _, _, _, _) = prepare_test_env();

        sleep(Duration::from_millis(10));
        let kalman_time = SystemTime::now();
        real_time_visualization.kalman_data.pop_front();
        real_time_visualization.kalman_data.push_back(Data {
            x: 33.3,
            y: 33.3,
            z: 33.3,
            timestamp: kalman_time,
        });

        assert_ne!(
            real_time_visualization.plot_stop,
            kalman_time
                .duration_since(real_time_visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
        real_time_visualization.update_plot_range();
        assert_eq!(
            real_time_visualization.plot_stop,
            kalman_time
                .duration_since(real_time_visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
    }

    #[test]
    fn test_update_plot_range_start_value() {
        let (mut real_time_visualization, _, _, _, _, _) = prepare_test_env();

        sleep(Duration::from_millis(100));
        let kalman_time = SystemTime::now();
        real_time_visualization.kalman_data.pop_front();
        real_time_visualization.kalman_data.pop_front();
        real_time_visualization.kalman_data.push_back(Data {
            x: 77.7,
            y: 77.7,
            z: 77.7,
            timestamp: SystemTime::now(),
        });
        real_time_visualization.kalman_data.push_front(Data {
            x: 33.3,
            y: 33.3,
            z: 33.3,
            timestamp: kalman_time,
        });

        assert_ne!(
            real_time_visualization.plot_start,
            kalman_time
                .duration_since(real_time_visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
        real_time_visualization.update_plot_range();
        assert_eq!(
            real_time_visualization.plot_start,
            kalman_time
                .duration_since(real_time_visualization.simulation_start)
                .unwrap()
                .as_millis()
        );
    }

    #[test]
    #[should_panic]
    fn test_update_plot_range_access_empty_buffer() {
        let (mut real_time_visualization, _, _, _, _, _) = prepare_test_env();

        real_time_visualization.kalman_data.clear();
        assert!(real_time_visualization.kalman_data.is_empty());

        real_time_visualization.update_plot_range();
    }
}
