use std::num::NonZeroU32;
use std::time::Duration;

pub fn get_cycle_duration(frequency: NonZeroU32) -> Duration {
    Duration::from_secs_f64(1.0 / frequency.get() as f64)
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
}
