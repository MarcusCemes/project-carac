use std::path::{Path, PathBuf};

use chunked_bytes::ChunkedBytes;
use eyre::{Result, bail};
use serde::{Deserialize, Serialize};
use tokio::{
    fs::{self, File},
    io::{self, AsyncReadExt, AsyncWriteExt},
};

use crate::{
    data::{experiment::Run, sink::StreamInfo},
    misc::buf::Encode,
};

use super::experiment::ExperimentHeader;

pub struct Session {
    metadata: SessionMetadata,
    open_experiment: Option<(File, String)>,
    root_path: PathBuf,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SessionMetadata {
    pub streams: Vec<StreamInfo>,
}

impl Session {
    const CHECKPOINTS_DIR: &str = "checkpoints";
    const EXPERIMENTS_DIR: &str = "experiments";
    const TEMP_EXPERIMENT_NAME: &str = "_current";

    /* == Accessors == */

    pub fn metadata(&self) -> &SessionMetadata {
        &self.metadata
    }

    /* == Persistence == */

    pub async fn open(path: PathBuf, streams: Vec<StreamInfo>) -> Result<Self> {
        fs::create_dir_all(&path).await?;

        let metadata = SessionMetadata { streams };
        metadata.save(&path).await?;

        let _ = fs::create_dir(path.join(Self::CHECKPOINTS_DIR)).await;
        let _ = fs::create_dir(path.join(Self::EXPERIMENTS_DIR)).await;

        Ok(Self {
            metadata,
            open_experiment: None,
            root_path: path,
        })
    }

    pub async fn new_experiment(&mut self, name: String) -> Result<()> {
        let path = self
            .root_path
            .join(Self::EXPERIMENTS_DIR)
            .join(Self::TEMP_EXPERIMENT_NAME);

        let mut file = File::create(path).await?;
        let mut buf = ChunkedBytes::new();

        let header = ExperimentHeader::new(name, self.metadata.channels());
        header.encode(&mut buf);

        file.write_all_buf(&mut buf).await?;

        self.open_experiment = Some((file, header.name));

        Ok(())
    }

    pub async fn append_run(&mut self, run: &Run) -> Result<()> {
        let Some((file, _)) = &mut self.open_experiment else {
            bail!("No experiment file open");
        };

        let mut buf = ChunkedBytes::new();
        run.encode(&mut buf);

        file.write_all_buf(&mut buf).await?;
        file.flush().await?;

        Ok(())
    }

    pub async fn save_experiment(&mut self) -> Result<()> {
        let Some((file, name)) = self.open_experiment.take() else {
            bail!("No experiment file open");
        };

        drop(file);

        let id = self.next_experiment_number().await?;
        let name = Self::experiment_name(id, &name);

        let dir = self.root_path.join(Self::EXPERIMENTS_DIR);
        let from = dir.join(Self::TEMP_EXPERIMENT_NAME);
        let to = dir.join(&name);

        fs::rename(from, to).await?;

        Ok(())
    }

    pub async fn list_experiments(&self) -> Result<Vec<(u32, PathBuf)>> {
        let dir = self.root_path.join(Self::EXPERIMENTS_DIR);
        let mut files = fs::read_dir(dir).await?;

        let mut experiments = Vec::new();

        while let Some(entry) = files.next_entry().await? {
            if let Some(name) = entry.file_name().to_str() {
                if let Some(id) = Self::extract_id(name) {
                    let path = entry.path();
                    experiments.push((id, path));
                }
            }
        }

        Ok(experiments)
    }

    /* == Misc == */

    pub fn experiment_name(id: u32, name: &str) -> String {
        format!("{id:04}_{name}")
    }

    pub fn output_name(id: u32, run: usize, extension: &str) -> String {
        format!("{id:04}_{run}.{extension}")
    }

    async fn next_experiment_number(&self) -> Result<u32> {
        let mut files = fs::read_dir(self.root_path.join(Self::EXPERIMENTS_DIR)).await?;
        let mut max_id = None;

        while let Some(entry) = files.next_entry().await? {
            if let Some(name) = entry.file_name().to_str() {
                if let Some(id) = Self::extract_id(name) {
                    let value = max_id.get_or_insert(id);
                    *value = (*value).max(id);
                }
            }
        }

        Ok(max_id.map(|id| id + 1).unwrap_or(0))
    }

    fn extract_id(name: &str) -> Option<u32> {
        let (id, _) = name.split_once('_')?;
        id.parse().ok()
    }
}

impl SessionMetadata {
    pub const FILE_NAME: &str = "_meta.json";
    const FIND_LEVELS: usize = 2;

    pub fn new(streams: Vec<StreamInfo>) -> Self {
        Self { streams }
    }

    pub async fn find(path: &Path) -> Option<Self> {
        let mut iter = path.ancestors();
        iter.next();

        for _ in 0..Self::FIND_LEVELS {
            match iter.next() {
                Some(path) => {
                    if let Ok(meta) = Self::load(path).await {
                        return Some(meta);
                    }
                }

                None => break,
            }
        }

        None
    }

    pub async fn load(path: &Path) -> Result<SessionMetadata> {
        let mut file = File::open(path.join(Self::FILE_NAME)).await?;

        let mut buf = String::new();
        file.read_to_string(&mut buf).await?;

        Ok(serde_json::from_str(&buf)?)
    }

    async fn save(&self, path: &Path) -> io::Result<()> {
        let mut file = File::create(path.join(Self::FILE_NAME)).await?;

        file.write_all(&serde_json::to_vec(self)?).await?;
        file.flush().await
    }

    fn channels(&self) -> Box<[u8]> {
        self.streams
            .iter()
            .map(|s| s.channels.len() as u8)
            .collect::<Vec<_>>()
            .into_boxed_slice()
    }
}
