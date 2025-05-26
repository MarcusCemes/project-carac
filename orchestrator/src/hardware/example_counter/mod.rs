use std::{fmt, sync::Arc, time::Duration};

use async_trait::async_trait;
use tokio::{
    sync::Mutex,
    task::JoinHandle,
    time::{Instant, interval},
};

use crate::data::sink::{DataSinkBuilder, StreamWriter};

use super::HardwareAgent;

pub struct ExampleCounter {
    inner: Arc<Mutex<Option<StreamWriter>>>,
    name: String,
    task: Option<JoinHandle<()>>,
}

impl ExampleCounter {
    pub fn new(name: String) -> Self {
        let inner = Arc::new(Mutex::new(None));
        let task = Some(tokio::spawn(Self::counter_task(inner.clone())));

        ExampleCounter { inner, name, task }
    }

    #[tracing::instrument(skip_all)]
    async fn counter_task(inner: Arc<Mutex<Option<StreamWriter>>>) {
        let mut counter = 0.;
        let mut clock = interval(Duration::from_millis(500));

        loop {
            clock.tick().await;
            let now = Instant::now();

            if let Some(stream) = inner.lock().await.as_ref() {
                stream.add(now, &[counter]).await;
            }

            counter += 1.;
        }
    }
}

#[async_trait]
impl HardwareAgent for ExampleCounter {
    async fn register(&mut self, sink: &mut DataSinkBuilder) {
        let name = self.name.clone();
        let channels = ["count"].map(str::to_owned).to_vec();
        let stream = sink.register_stream(name, channels).await;

        *self.inner.lock().await = Some(stream);
    }
}

impl fmt::Display for ExampleCounter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Counter ({})", self.name)
    }
}

impl Drop for ExampleCounter {
    fn drop(&mut self) {
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }
}
