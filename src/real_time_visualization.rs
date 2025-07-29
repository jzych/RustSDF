use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::coord::types::{RangedCoordf64, RangedCoordi128};
use plotters::coord::Shift;
use plotters::prelude::*;
use plotters_piston::{draw_piston_window, PistonBackend};

use crate::config;
use crate::data::{Data, Telemetry};
use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::time::SystemTime;

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
    plot_start: i128,
    plot_stop: i128,
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
                [Data::new(); (config::PLOT_RANGE_WINDOW * config::GPS_FREQ.get()) as usize],
            ),
            avg_data: VecDeque::from(
                [Data::new(); (config::PLOT_RANGE_WINDOW * config::GPS_FREQ.get()) as usize],
            ),
            kalman_data: VecDeque::from(
                [Data::new(); (config::PLOT_RANGE_WINDOW * config::IMU_FREQ.get()) as usize],
            ),
            inertial_data: VecDeque::from(
                [Data::new(); (config::PLOT_RANGE_WINDOW * config::IMU_FREQ.get()) as usize],
            ),
            groundtruth_data: VecDeque::from(
                [Data::new(); (config::PLOT_RANGE_WINDOW * config::GENERATOR_FREQ.get()) as usize],
            ),
            rx_gps,
            rx_avg,
            rx_kalman,
            rx_inertial,
            rx_groundtruth,
            plot_start: SystemTime::now()
                .duration_since(simulation_start)
                .unwrap()
                .as_millis() as i128,
            plot_stop: SystemTime::now()
                .duration_since(simulation_start)
                .unwrap()
                .as_millis() as i128,
            simulation_start,
        }
    }

    pub fn run(
        rx_gps: Receiver<Telemetry>,
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,
        rx_inertial: Receiver<Telemetry>,
        rx_groundtruth: Receiver<Telemetry>,
        simulation_start: SystemTime,
    ) {
        let mut window: PistonWindow = WindowSettings::new("RustSFD", [450, 300])
            .samples(4)
            .exit_on_esc(true)
            .build()
            .unwrap();

        window.set_max_fps(config::FPS as u64);

        let mut real_time_visualization = RealTimeVisualization::new(
            rx_gps,
            rx_avg,
            rx_kalman,
            rx_inertial,
            rx_groundtruth,
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
            BLACK,
            &mut chart,
            coord,
        );
        self.chart_data(
            &self.gps_data,
            "GPS data",
            RGBColor(150,150,150),
            &mut chart,
            coord
        );
        self.chart_data(
            &self.avg_data,
            "Moving average filter",
            BLUE,
            &mut chart,
            coord,
        );
        self.chart_data(
            &self.inertial_data,
            "Inertial navigator",
            RGBColor(0,225,0),
            &mut chart,
            coord,
        );
        self.chart_data(
            &self.kalman_data,
            "Kalman filter",
            RED,
            &mut chart,
            coord
        );

        chart
            .configure_series_labels()
            .border_style(BLACK)
            .background_style(WHITE.mix(0.0))
            .draw()
            .unwrap();
    }

    fn update_plot_range(&mut self) {
        self.plot_stop = match self.kalman_data.back() {
            Some(data) => data
                .timestamp
                .duration_since(self.simulation_start)
                .unwrap()
                .as_millis() as i128,
            None => panic!("Trying to access empty buffer!"),
        };

        if (self.plot_stop - self.plot_start) > (config::PLOT_RANGE_WINDOW as i128) {
            self.plot_start = self
                .kalman_data
                .front()
                .unwrap()
                .timestamp
                .duration_since(self.simulation_start)
                .unwrap()
                .as_millis() as i128;
        }
    }

    fn chart_data(
        &self,
        data: &VecDeque<Data>,
        label: &str,
        color: RGBColor,
        chart: &mut ChartContext<
            '_,
            PistonBackend<'_, '_>,
            Cartesian2d<RangedCoordi128, RangedCoordf64>,
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
                            .as_millis() as i128,
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
