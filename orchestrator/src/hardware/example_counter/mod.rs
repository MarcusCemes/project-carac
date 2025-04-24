use std::time::Duration;

use tokio::{task::JoinHandle, time::interval};

use crate::recording::{Recorder, StreamHandle};

#[derive(Default)]
pub struct ExampleCounter {
    task: Option<JoinHandle<()>>,
}

impl ExampleCounter {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn subscribe<T: ToString>(&mut self, recorder: &Recorder, name: T) {
        let stream = recorder.add_stream(name, &["count"]).await;
        let new_task = tokio::spawn(Self::publisher_task(stream));

        if let Some(task) = self.task.replace(new_task) {
            task.abort();
        }
    }

    #[tracing::instrument(skip(stream))]
    async fn publisher_task(stream: StreamHandle) {
        let mut counter = 0.;
        let mut clock = interval(Duration::from_millis(500));

        loop {
            clock.tick().await;
            stream.add(&[counter]).await;
            counter += 1.;
        }
    }
}

impl Drop for ExampleCounter {
    fn drop(&mut self) {
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }
}
