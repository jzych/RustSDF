pub mod real_time_visualization;
pub mod static_visualization;

use crate::data::{Data, Telemetry};
use std::sync::mpsc::Receiver;

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