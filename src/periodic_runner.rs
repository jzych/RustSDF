use std::{
    error::Error,
    time::{Duration, Instant},
};

///
/// Function that can run Callable periodically with high precision.
/// Due to implementation with spin-loop for the whole duration of the wait
/// it is preferred to use it for running high frequency tasks.
///
pub fn run_periodicaly(
    mut runnable: impl FnMut() -> Result<(), Box<dyn Error>>,
    mut stop_condition: impl FnMut() -> bool,
    period: Duration,
) -> Result<(), Box<dyn Error>> {
    assert!(
        !period.is_zero(),
        "Specified period must be greater than zero."
    );

    let mut time_point = Instant::now();
    while !stop_condition() {
        runnable()?;
        while time_point.elapsed() < period {
            std::hint::spin_loop();
        }
        time_point = Instant::now();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use assertables::*;
    use std::time::Duration;
    use flaky_test::flaky_test;

    use super::*;

    #[test]
    #[should_panic]
    fn given_zero_period_expect_panic() {
        let zero_period = Duration::new(0, 0);
        let _ = run_periodicaly(|| Ok(()), || false, zero_period);
    }

    #[test]
    fn given_stop_condition_true_expect_function_to_exit() {
        let mut counter = 0;
        let stop_after_three_cycles = || {
            counter += 1;
            counter == 4
        };
        let arbitrary_running_period = Duration::from_millis(10);

        let result = run_periodicaly(|| Ok(()), stop_after_three_cycles, arbitrary_running_period);

        assert!(result.is_ok());
    }

    // Given that we are running on general purpose OS, not RTOS
    // we cannot expect that this test will pass on first try.
    // Two tries should suffice, but on CI's VM higher number is necessary.
    #[test]
    #[flaky_test]
    fn given_period_expect_runner_will_sleep_with_given_time() {
        let mut counter = 0;
        let stop_after_a_cycle = || {
            counter += 1;
            counter == 2
        };
        let arbitrary_running_period = Duration::from_millis(10);
        let start = Instant::now();

        let result = run_periodicaly(|| Ok(()), stop_after_a_cycle, arbitrary_running_period);

        assert!(result.is_ok());
        let running_time = start.elapsed();
        let precision = 0.2;
        let difference = arbitrary_running_period.as_secs_f64() * precision;
        assert_lt!(
            running_time.as_secs_f64(),
            arbitrary_running_period.as_secs_f64() + difference
        );
        assert_gt!(
            running_time.as_secs_f64(),
            arbitrary_running_period.as_secs_f64()
        );
    }

    #[derive(Debug)]
    struct TestError;
    impl std::fmt::Display for TestError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "TestError")
        }
    }
    impl std::error::Error for TestError {}

    #[test]
    fn given_called_function_returns_error_expect_exiting_loop_with_error() {
        let never_stops = || false;
        let arbitrary_running_period = Duration::from_millis(10);
        let returns_error = || -> Result<(), Box<dyn Error>> { Err(Box::new(TestError)) };

        let result = run_periodicaly(returns_error, never_stops, arbitrary_running_period);
        assert!(result.is_err());
    }
}
