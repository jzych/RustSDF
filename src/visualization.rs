pub mod real_time_visualization;
pub mod static_visualization;

use crate::config;
use crate::StaticVisualization;
use crate::RealTimeVisualization;
use crate::data::{Data, Telemetry};
use plotters::coord::types::{RangedCoordf64, RangedCoordu128};
use plotters::coord::Shift;
use plotters::prelude::*;
use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::time::SystemTime;

enum VisualizationType {
    Static,
    Dynamic,
}

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

pub struct Visualization {
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

impl Visualization {
    fn get_plot_data(&mut self, plot_data_type: PlotDataType, visualization_type: VisualizationType) {
        let (rx, rx_data) = match plot_data_type {
            PlotDataType::Gps => (&self.rx_gps, &mut self.gps_data),
            PlotDataType::Avg => (&self.rx_avg, &mut self.avg_data),
            PlotDataType::Kalman => (&self.rx_kalman, &mut self.kalman_data),
            PlotDataType::Inertial => (&self.rx_inertial, &mut self.inertial_data),
            PlotDataType::Groundtruth => (&self.rx_groundtruth, &mut self.groundtruth_data),
        };

        match visualization_type {
            VisualizationType::Static => {
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
            VisualizationType::Dynamic => {
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
        }

        // while let Ok(data) = match visualization_type {
        //     VisualizationType::Static => rx.recv().unwrap(),
        //     VisualizationType::Dynamic => rx.try_recv().unwrap(),
        // } {
        //     match data {
        //         Telemetry::Position(d) => {
        //             rx_data.pop_front();
        //             rx_data.push_back(d);
        //         }
        //         Telemetry::Acceleration(_) => {
        //             panic!("Acceleration should not be passed as an input!");
        //         }
        //     }
        // }
    }

    fn draw_coordinate<DB>(&mut self, root: DrawingArea<DB, Shift>, coord: PlotAxis)
    where
        DB: DrawingBackend,
    {
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

    fn chart_data<'a, DB>(
        &self,
        data: &VecDeque<Data>,
        label: &str,
        color: RGBColor,
        chart: &mut ChartContext<'a, DB, Cartesian2d<RangedCoordu128, RangedCoordf64>>,
        coord: PlotAxis,
    ) where
        DB: DrawingBackend,
    {
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

// trait VisualizationOperation {
//     fn new(
//         rx_gps: Receiver<Telemetry>,
//         rx_avg: Receiver<Telemetry>,
//         rx_kalman: Receiver<Telemetry>,
//         rx_inertial: Receiver<Telemetry>,
//         rx_groundtruth: Receiver<Telemetry>,
//         simulation_start: SystemTime,
//     );
//     fn run(receivers: PlotterReceivers, simulation_start: SystemTime);
//     fn update_plot_range(&mut self);
// }

// impl VisualizationOperation for VisualizationType {
//     fn new(
//             rx_gps: Receiver<Telemetry>,
//             rx_avg: Receiver<Telemetry>,
//             rx_kalman: Receiver<Telemetry>,
//             rx_inertial: Receiver<Telemetry>,
//             rx_groundtruth: Receiver<Telemetry>,
//             simulation_start: SystemTime,
//         ) {
//         match self {
//             VisualizationType::Static(visualization) => StaticVisualization::new(rx_gps, rx_avg, rx_kalman, rx_inertial, rx_groundtruth, simulation_start),
//             VisualizationType::Dynamic(visualization) => RealTimeVisualization::new(rx_gps, rx_avg, rx_kalman, rx_inertial, rx_groundtruth, simulation_start),
//         }
//     }
// }
