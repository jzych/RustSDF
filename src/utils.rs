use std::num::NonZeroU32;
use std::time::Duration;

use plotters::style::RGBColor;

pub const GROUNDTURTH_PLOT_COLOR: RGBColor = RGBColor(0, 0, 0);     //black
pub const GPS_PLOT_COLOR: RGBColor = RGBColor(150, 150, 150);       //light grey
pub const AVERAGE_PLOT_COLOR: RGBColor = RGBColor(0, 0, 255);       //blue
pub const INERTIAL_PLOT_COLOR: RGBColor = RGBColor(0, 225, 0);      //dark green
pub const KALMAN_PLOT_COLOR: RGBColor = RGBColor(255, 0, 0);        //red

pub fn get_cycle_duration(frequency: NonZeroU32) -> Duration {
    Duration::from_secs_f64(get_cycle_duration_f64(frequency))
}

pub fn get_cycle_duration_f64(frequency: NonZeroU32) -> f64 {
    1.0 / frequency.get() as f64
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn given_frequency_expect_cycle_duration() {
        let frequency = NonZeroU32::new(10).unwrap();
        let expected_cycle_duration = Duration::from_millis(100);
        assert_eq!(get_cycle_duration(frequency), expected_cycle_duration);
    }

    #[test]
    fn given_frequency_expect_cycle_duration_f64() {
        let frequency = NonZeroU32::new(10).unwrap();
        let expected_cycle_duration = 0.1;
        assert_eq!(get_cycle_duration_f64(frequency), expected_cycle_duration);
    }
}
