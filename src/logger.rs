use std::{
    any::Any,
    collections::HashMap,
    fmt::Debug,
    sync::{mpsc, Arc, Mutex},
    thread,
    time::SystemTime,
};

use once_cell::sync::Lazy;

#[derive(Debug, Clone)]
pub struct LogEntry<T> {
    pub timestamp: SystemTime,
    pub data: T,
}

pub struct Logger<T: Send + Clone + Debug + 'static> {
    sender: mpsc::Sender<LogEntry<T>>, 
    data_storage: Arc<Mutex<Vec<LogEntry<T>>>>,
}

impl<T: Send + Clone + Debug + 'static> Logger<T> {
    fn new() -> Self {
        let (tx, rx) = mpsc::channel::<LogEntry<T>>();
        let data_storage = Arc::new(Mutex::new(Vec::new()));
        let storage_clone = Arc::clone(&data_storage);

        thread::spawn(move || {
            for entry in rx {
                let mut storage = storage_clone.lock().unwrap();
                storage.push(entry);
            }
        });

        Logger {
            sender: tx,
            data_storage,
        }
    }

    fn log(&self, data: T) {
        let entry = LogEntry {
            timestamp: SystemTime::now(),
            data,
        };
        let _ = self.sender.send(entry);
    }

    fn get_data(&self) -> Vec<LogEntry<T>> {
        self.data_storage.lock().unwrap().clone()
    }
}

static LOGGERS: Lazy<Mutex<HashMap<String, Box<dyn Any + Send + Sync>>>> =
    Lazy::new(|| Mutex::new(HashMap::new()));

fn get_or_create_logger<T: Send + Clone + Debug + Sync + 'static>(name: &str) -> Arc<Logger<T>> {
    let mut loggers = LOGGERS.lock().unwrap();

    if let Some(logger) = loggers.get(name) {
        if let Some(typed_logger) = logger.downcast_ref::<Arc<Logger<T>>>() {
            return typed_logger.clone();
        }
    }

    let new_logger = Arc::new(Logger::<T>::new());
    loggers.insert(name.to_string(), Box::new(new_logger.clone()));
    new_logger
}

pub fn log<T: Send + Clone + Debug + Sync + 'static>(component_name: &str, data: T) {
    let logger = get_or_create_logger::<T>(component_name);
    logger.log(data);
}

pub fn get_data<T: Send + Clone + Debug + Sync + 'static>(
    component_name: &str,
) -> Option<Vec<LogEntry<T>>> {
    let loggers = LOGGERS.lock().unwrap();
    if let Some(logger) = loggers.get(component_name) {
        if let Some(typed_logger) = logger.downcast_ref::<Arc<Logger<T>>>() {
            return Some(typed_logger.get_data());
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_single_component_log() {
        log("test_component", "test message");
        thread::sleep(Duration::from_millis(10));
        let data = get_data::<&str>("test_component").unwrap();
        assert_eq!(data.len(), 1);
        assert_eq!(data[0].data, "test message");
    }

    #[test]
    fn test_no_call_to_log_for_component() {
        let result = get_data::<String>("some_coponent");
        assert!(
            result.is_none(),
            "Result shall be None when component has not logged anything"
        );
    }

    #[test]
    fn test_multiple_components_log() {
        log("string_component", "string data");
        log("number_component", 42);
        thread::sleep(Duration::from_millis(10));

        let string_data = get_data::<&str>("string_component").unwrap();
        let number_data = get_data::<i32>("number_component").unwrap();

        assert_eq!(string_data[0].data, "string data");
        assert_eq!(number_data[0].data, 42);
    }
}
