use std::path::{Path, PathBuf};

use chunked_bytes::ChunkedBytes;
use eyre::{Context, Result, bail};
use tokio::{
    fs::{File, create_dir_all},
    io::{AsyncReadExt, AsyncWriteExt},
};

use crate::{
    data::{
        experiment::{Experiment, Run},
        sink::StreamInfo,
    },
    misc::standard_config,
};

use super::experiment::ExperimentHeader;

const META_FILE: &str = "_meta.json";

pub struct Session {
    experiment: Option<File>,
    experiment_counter: u32,
    path: PathBuf,
    streams: Vec<StreamInfo>,
}

impl Session {
    pub async fn read_metadata(path: &Path) -> Result<Vec<StreamInfo>> {
        let mut file = File::open(path.join(META_FILE))
            .await
            .wrap_err("Failed to open meta file")?;

        let mut buf = String::new();
        file.read_to_string(&mut buf).await?;

        Ok(serde_json::from_str(&buf)?)
    }

    pub async fn create(path: PathBuf, streams: Vec<StreamInfo>) -> Result<Self> {
        create_dir_all(&path).await?;

        let mut file = File::create_new(path.join(META_FILE))
            .await
            .wrap_err("Failed to create meta file")?;

        file.write_all(&serde_json::to_vec(&streams)?).await?;
        file.flush().await?;

        Ok(Session {
            experiment: None,
            experiment_counter: 0,
            path,
            streams,
        })
    }

    pub async fn read_experiment(path: &Path, id: u32) -> Result<Experiment> {
        let filename = format!("experiment_{}", id);
        let path = path.join(&filename);

        let mut buf = Vec::new();
        let mut file = File::open(path).await?;

        file.read_to_end(&mut buf).await?;

        Experiment::decode(&mut &buf[..])
    }

    pub async fn create_experiment(&mut self, name: String) -> Result<()> {
        let file_name = format!("experiment_{}", self.experiment_counter);
        let path = self.path.join(&file_name);
        let mut file = File::create_new(&path).await?;

        let header = ExperimentHeader::new(name, self.stream_channels());
        let header = bincode::encode_to_vec(&header, standard_config()).unwrap();

        file.write_all(&header).await?;

        self.experiment = Some(file);
        self.experiment_counter += 1;

        Ok(())
    }

    pub fn close_experiment(&mut self) {
        self.experiment = None;
    }

    pub async fn add_run(&mut self, run: &Run) -> Result<()> {
        let Some(experiment) = &mut self.experiment else {
            bail!("No experiment file open");
        };

        let mut buf = ChunkedBytes::new();
        run.encode(&mut buf)?;

        experiment.write_all_buf(&mut buf).await?;
        experiment.flush().await?;

        Ok(())
    }

    pub fn streams(&self) -> &[StreamInfo] {
        &self.streams
    }

    fn stream_channels(&self) -> Box<[u8]> {
        self.streams
            .iter()
            .map(|s| s.channels.len() as u8)
            .collect::<Vec<_>>()
            .into_boxed_slice()
    }
}
