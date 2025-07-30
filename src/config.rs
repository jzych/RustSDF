use std::num::NonZeroU32;
use plotters::style::RGBColor;
// Here are stored configuration values for the project

// Refresh rate for tasks in Hz
pub const GENERATOR_FREQ: NonZeroU32 = NonZeroU32::new(100).unwrap();
pub const IMU_FREQ: NonZeroU32 = NonZeroU32::new(20).unwrap();
pub const GPS_FREQ: NonZeroU32 = NonZeroU32::new(5).unwrap();

// Trajectory generator config
pub const HELIX_FREQUENCY: f64 = 0.5;

// Sensor output noise parameters
pub const GPS_OUTPUT_NOISE_SIGMA: f64 = 10.0;
pub const IMU_OUTPUT_NOISE_SIGMA: f64 = 1.0;

// Kalman tuning parameters
pub const KALMAN_GPS_SIGMA: f64 = 10.0;
pub const KALMAN_ACC_SIGMA: f64 = 1.0;
pub const KALMAN_TIMING_TOLERANCE: f64 = 0.02; // 0.01 = 1% of timing tolerance

// Visualiziation parameters
pub const FPS: u32 = 5;
pub const PLOT_RANGE_WINDOW: u128 = 15;
pub const PLOT_RANGE_Y_AXIS_MIN: f64 = -20.0;
pub const PLOT_RANGE_Y_AXIS_MAX: f64 = 120.0;

// Average filter tuning parameters
pub const BUFFER_LENGTH: usize = 3;

// Plot chart colors
pub const GROUNDTRUTH_PLOT_COLOR: RGBColor = RGBColor(0, 0, 0);     // black
pub const GPS_PLOT_COLOR: RGBColor = RGBColor(150, 150, 150);       // light grey
pub const AVERAGE_PLOT_COLOR: RGBColor = RGBColor(0, 0, 255);       // blue
pub const INERTIAL_PLOT_COLOR: RGBColor = RGBColor(0, 225, 0);      // dark green
pub const KALMAN_PLOT_COLOR: RGBColor = RGBColor(255, 0, 0);        // red
