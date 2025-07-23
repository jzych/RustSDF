use std::num::NonZeroU32;
// Here are stored configuration values for the project
pub const SIMULATION_TIME: u64 = 20;

// Refresh rate for tasks in Hz
pub const GENERATOR_FREQ: NonZeroU32 = NonZeroU32::new(100).unwrap();
pub const IMU_FREQ: NonZeroU32 = NonZeroU32::new(20).unwrap();
pub const GPS_FREQ: NonZeroU32 = NonZeroU32::new(5).unwrap();

// Trajectory generator config
pub const HELIX_FREQUENCY: f64 = 0.5;
