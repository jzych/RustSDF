use crate::data::{Data, Telemetry};
use crate::log_config::*;
use crate::logger::get_data;
use csv::Writer;
use std::fmt::Debug;
use std::fs::create_dir_all;
use std::path::{Path, PathBuf};
use std::{error::Error, fs::OpenOptions};

const OUTPUT_PATH: &str = "output";
const CSV_EXTENSION: &str = "csv";

fn save_log_to_file<T: serde::Serialize + Send + Clone + Debug + Sync + 'static>(
    path: &str,
    component_name: &str,
) -> Result<(), Box<dyn Error>> {
    let parent = Path::new(path).parent().ok_or("Cannot create directory.")?;

    create_dir_all(parent)?;

    let file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(path)?;
    let mut writter = Writer::from_writer(file);
    let logged_component = get_data::<T>(component_name);

    if let Some(data) = logged_component {
        for entry in data {
            writter.serialize(&entry.data)?;
        }
        writter.flush()?;
    } else {
        eprintln!("Logged {component_name} data not found")
    }

    Ok(())
}

fn save_log_handle<T: serde::Serialize + Send + Clone + Debug + Sync + 'static>(
    path: &str,
    component_name: &str,
) {
    match save_log_to_file::<T>(path, component_name) {
        Ok(_) => println!("{component_name} data saved to {path}"),
        Err(e) => eprintln!("Error {e}"),
    }
}

#[inline]
fn concat_path(component_name: &str) -> String {
    let file_name = component_name.to_lowercase();
    let mut path = PathBuf::from(OUTPUT_PATH);
    path.push(format!("{file_name}.{CSV_EXTENSION}"));
    path.to_string_lossy().into_owned()
}

fn save_imu_log_to_file() {
    save_log_handle::<Data>(concat_path(IMU_LOG).as_str(), IMU_LOG);
}

fn save_gps_log_to_file() {
    save_log_handle::<Data>(concat_path(GPS_LOG).as_str(), GPS_LOG);
}

fn save_inertial_nav_to_file() {
    save_log_handle::<Telemetry>(
        concat_path(INTERTIAL_NAVIGATOR_LOG).as_str(),
        INTERTIAL_NAVIGATOR_LOG,
    );
}

fn save_kalman_log_to_file() {
    save_log_handle::<Telemetry>(concat_path(KALMAN_LOG).as_str(), KALMAN_LOG);
}

fn save_general_log_to_file() {
    save_log_handle::<String>(concat_path(GENERAL_LOG).as_str(), GENERAL_LOG);
}

fn save_groundtruth_log_to_file() {
    save_log_handle::<Data>(concat_path(GROUNDTRUTH_LOG).as_str(), GROUNDTRUTH_LOG);
}

fn save_moving_average_log_to_file() {
    save_log_handle::<Data>(concat_path(MOVING_AVERAGE_LOG).as_str(), MOVING_AVERAGE_LOG);
}

pub fn save_logs_to_file() {
    save_gps_log_to_file();
    save_imu_log_to_file();
    save_inertial_nav_to_file();
    save_kalman_log_to_file();
    save_general_log_to_file();
    save_groundtruth_log_to_file();
    save_moving_average_log_to_file();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::logger::log;
    use std::fs;
    use std::io::Read;
    use std::time::SystemTime;

    const TEST_PATH: &str = "test_output/test_log.csv";
    const TEST_NO_DATA_PATH: &str = "test_output_no_data/test_log.csv";
    const TEST_COMPONENT: &str = "TEST_COMPONENT";
    const EMPTY_TEST_COMPONENT: &str = "EMPTY_TEST_COMPONENT";

    fn create_test_data() -> Data {
        Data {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            timestamp: SystemTime::now(),
        }
    }

    fn cleanup_test_directory() {
        let _ = fs::remove_dir_all("test_output");
    }

    fn cleanup_test_file(component_name: &str) {
        let _ = fs::remove_file(concat_path(component_name));
    }

    fn read_file_content(path: &str) -> String {
        let mut file = fs::File::open(path).unwrap();
        let mut content = String::new();
        file.read_to_string(&mut content).unwrap();
        content
    }

    #[test]
    fn test_concat_path() {
        let expected_path = PathBuf::from(OUTPUT_PATH)
            .join(format!(
                "{}.{}",
                TEST_COMPONENT.to_lowercase(),
                CSV_EXTENSION
            ))
            .to_string_lossy()
            .into_owned();

        let result = concat_path(TEST_COMPONENT);

        assert_eq!(result, expected_path);
    }

    #[test]
    fn test_save_log_to_file_no_data() {
        let result = save_log_to_file::<Data>(TEST_NO_DATA_PATH, EMPTY_TEST_COMPONENT);

        assert!(result.is_ok());

        assert!(Path::new(TEST_NO_DATA_PATH).exists());
        let content = read_file_content(TEST_NO_DATA_PATH);
        assert!(content.is_empty());

        let _ = fs::remove_dir_all("test_output_no_data");
    }

    #[test]
    fn test_save_log_to_file_with_data() {
        let test_data = create_test_data();
        log(TEST_COMPONENT, test_data);

        let result = save_log_to_file::<Data>(TEST_PATH, TEST_COMPONENT);

        assert!(result.is_ok());
        assert!(Path::new(TEST_PATH).exists());

        let content = read_file_content(TEST_PATH);
        assert!(!content.is_empty());

        cleanup_test_directory();
    }

    #[test]
    fn test_invalid_path() {
        let invalid_path = format!("test_output/{}invalid|path.csv", '\0');
        let test_data = create_test_data();
        log(TEST_COMPONENT, test_data);

        let result = save_log_to_file::<Data>(invalid_path.as_str(), TEST_COMPONENT);
        assert!(result.is_err());
    }

    macro_rules! test_x_log_to_file {
        ($($name:ident : ($function:expr,$component_name:expr)),+) => {
        $(
            #[test]
            fn $name() {
                $function();

                assert!(Path::new(&concat_path($component_name)).exists());
                cleanup_test_file($component_name);
            }
        )+
        }
    }

    test_x_log_to_file! {
        test_save_general_log_to_file: (save_general_log_to_file, GENERAL_LOG),
        test_save_gps_log_to_file: (save_gps_log_to_file, GPS_LOG),
        test_save_groundtruth_log_to_file: (save_groundtruth_log_to_file, GROUNDTRUTH_LOG),
        test_save_imu_log_to_file: (save_imu_log_to_file, IMU_LOG),
        test_save_inertial_nav_to_file: (save_inertial_nav_to_file, INTERTIAL_NAVIGATOR_LOG),
        test_save_kalman_log_to_file: (save_kalman_log_to_file, KALMAN_LOG),
        test_save_moving_average_log_to_file: (save_moving_average_log_to_file, MOVING_AVERAGE_LOG)
    }
}
