use std::time::SystemTime;

mod string_timestamp {
    use chrono::{DateTime, Local};
    use serde::Serializer;
    use std::time::SystemTime;

    pub fn serialize<S>(time: &SystemTime, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let datetime: DateTime<Local> = (*time).into();
        let formatted = datetime.format("%Y-%m-%d %H:%M:%S%.3f").to_string();
        serializer.serialize_str(&formatted)
    }
}

#[derive(Debug, Copy, Clone, serde::Serialize)]
pub struct Data {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    #[serde(with = "string_timestamp")]
    pub timestamp: SystemTime,
}

impl Data {
    pub fn new() -> Self {
        Data {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            timestamp: SystemTime::now(),
        }
    }
}

impl Default for Data {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Copy, Clone, serde::Serialize)]
pub enum Telemetry {
    Acceleration(Data),
    Position(Data),
}

impl Telemetry {
    pub fn data(&self) -> &Data {
        match self {
            Telemetry::Acceleration(d) | Telemetry::Position(d) => d,
        }
    }
}
