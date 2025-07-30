#[derive(Debug)]
pub struct NoSubscribers;

impl std::fmt::Display for NoSubscribers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "No subscribers for IMU output.")
    }
}

impl std::error::Error for NoSubscribers {}
