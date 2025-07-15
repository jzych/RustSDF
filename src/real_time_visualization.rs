use local_pp::{draw_piston_window, PistonBackend};
use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::coord::types::RangedCoordf64;
use plotters::prelude::*;

use crate::config;
use crate::data::{Data, Telemetry};
use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::time::UNIX_EPOCH;

enum PlotDataType {
    Gps,
    Avg,
    Kalman,
}

pub struct RealTimeVisualization {
    gps_data: VecDeque<Data>,
    avg_data: VecDeque<Data>,
    kalman_data: VecDeque<Data>,
    rx_gps: Receiver<Telemetry>,
    rx_avg: Receiver<Telemetry>,
    rx_kalman: Receiver<Telemetry>,
}

impl RealTimeVisualization {
    fn new(
        rx_gps: Receiver<Telemetry>,
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,
    ) -> RealTimeVisualization {
        RealTimeVisualization {
            gps_data: VecDeque::from(
                [Data::new(); (config::FPS * config::GPS_FREQ.get()) as usize],
            ),
            avg_data: VecDeque::from(
                [Data::new(); (config::FPS * config::IMU_FREQ.get()) as usize],
            ),
            kalman_data: VecDeque::from(
                [Data::new(); (config::FPS * config::IMU_FREQ.get()) as usize],
            ),
            rx_gps,
            rx_avg,
            rx_kalman,
        }
    }

    pub fn run(
        rx_gps: Receiver<Telemetry>,
        rx_avg: Receiver<Telemetry>,
        rx_kalman: Receiver<Telemetry>,
    ) {
        let mut window: PistonWindow = WindowSettings::new("RustSFD", [450, 300])
            .samples(4)
            .exit_on_esc(true)
            .build()
            .unwrap();

        window.set_max_fps(config::FPS as u64);

        let mut real_time_visualization = RealTimeVisualization::new(rx_gps, rx_avg, rx_kalman);

        while draw_piston_window(&mut window, |b: PistonBackend<'_, '_>| {
            real_time_visualization.get_plot_data(PlotDataType::Gps);
            real_time_visualization.get_plot_data(PlotDataType::Avg);
            real_time_visualization.get_plot_data(PlotDataType::Kalman);
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

    fn draw(&self, b: PistonBackend<'_, '_>) {
        let root: DrawingArea<PistonBackend<'_, '_>, _> = b.into_drawing_area();
        let _ = root.fill(&WHITE);

        let (upper, lower) = root.split_vertically(40);
        upper.titled("RustSDF", ("comic-sans", 30)).unwrap();

        let split_in_3 = lower.split_evenly((3, 1));
        let (_, _lower_0) = split_in_3[0].split_vertically(40);
        let (_, _lower_1) = split_in_3[1].split_vertically(40);
        let (_, _lower_2) = split_in_3[2].split_vertically(40);

        let plot_start = self.gps_data[0]
            .timestamp
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as f64;

        let plot_stop = self.gps_data[4]
            .timestamp
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as f64;

        let mut chart = ChartBuilder::on(&root)
            .x_label_area_size(40)
            .y_label_area_size(50)
            .right_y_label_area_size(60)
            .margin_bottom(30)
            .build_cartesian_2d(plot_start..plot_stop, -10f64..200f64)
            .unwrap();

        chart
            .configure_mesh()
            .x_desc("time")
            .y_desc("x")
            .draw()
            .unwrap();

        self.chart_data(&self.gps_data, "GPS real data", GREEN, &mut chart);
        self.chart_data(&self.avg_data, "Moving average filter", BLUE, &mut chart);
        self.chart_data(&self.kalman_data, "Kalman filter", RED, &mut chart);

        chart
            .configure_series_labels()
            .border_style(BLACK)
            .background_style(WHITE.mix(0.0))
            .draw()
            .unwrap();
    }

    fn chart_data(
        &self,
        data: &VecDeque<Data>,
        label: &str,
        color: RGBColor,
        chart: &mut ChartContext<
            '_,
            PistonBackend<'_, '_>,
            Cartesian2d<RangedCoordf64, RangedCoordf64>,
        >,
    ) {
        chart
            .draw_series(LineSeries::new(
                data.iter().map(|p| {
                    (
                        p.timestamp.duration_since(UNIX_EPOCH).unwrap().as_millis() as f64,
                        p.x,
                    )
                }),
                color,
            ))
            .unwrap()
            .label(label)
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));
    }
}
