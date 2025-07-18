use std::time::Duration;

pub fn get_cycle_duration(frequency: u32) -> Duration {
    assert!(frequency != 0);
    Duration::from_secs_f64(1.0 / frequency as f64)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn given_frequency_expect_cycle_duration() {
        let frequency = 10;
        let expected_cycle_duration = Duration::from_millis(100);
        assert_eq!(get_cycle_duration(frequency), expected_cycle_duration);
    }

    #[test]
    #[should_panic]
    fn given_zero_frequency_expect_panic() {
        let frequency = 0;
        get_cycle_duration(frequency);
    }
}
