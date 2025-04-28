use std::time::Duration;

use eyre::Result;
use tokio::{task::JoinHandle, time::interval};

use crate::recording::{Sink, StreamWriter};

#[derive(Default)]
pub struct ExampleCounter {
    task: Option<JoinHandle<()>>,
}

impl ExampleCounter {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn subscribe(&mut self, sink: &Sink, name: String) -> Result<()> {
        let channels = ["count"].map(str::to_owned).to_vec();
        let stream = sink.add_stream(name, channels).await?;
        let new_task = tokio::spawn(Self::counter_task(stream));

        if let Some(task) = self.task.replace(new_task) {
            task.abort();
        }

        Ok(())
    }

    #[tracing::instrument(skip_all)]
    async fn counter_task(stream: StreamWriter) {
        let mut counter = 0.;
        let mut clock = interval(Duration::from_millis(500));

        loop {
            clock.tick().await;
            stream.write_now(&[counter]).await;
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
