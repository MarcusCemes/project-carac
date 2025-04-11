use std::time::Duration;

use tokio::{spawn, task::JoinHandle, time::interval};

use crate::recorder::RecordHandle;

const DIMENSIONS: usize = 2;

pub struct MockCounter {
    task: JoinHandle<()>,
}

impl MockCounter {
    pub async fn create(stream: RecordHandle<DIMENSIONS>) -> Self {
        let task = spawn(Self::publish_task(stream));
        Self { task }
    }

    async fn publish_task(stream: RecordHandle<DIMENSIONS>) {
        tracing::info!("Publisher task started");

        let mut counter = 0.;
        let mut clock = interval(Duration::from_millis(100));

        loop {
            stream.append([counter, counter + 0.5]).await;
            counter += 1.;
            clock.tick().await;
        }
    }
}

impl Drop for MockCounter {
    fn drop(&mut self) {
        tracing::warn!("Stopping counter task");
        self.task.abort();
    }
}
