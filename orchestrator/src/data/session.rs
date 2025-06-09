use std::{
    fs::{File, create_dir_all, read_dir, rename},
    io::{self, Write},
    path::{Path, PathBuf},
};

use chunked_bytes::ChunkedBytes;
use eyre::{Result, bail};
use serde::{Deserialize, Serialize};

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
    const EXPERIMENTS_DIR: &str = "experiments";
    const TEMP_EXPERIMENT_NAME: &str = "_current";

    /* == Accessors == */

    pub fn metadata(&self) -> &SessionMetadata {
        &self.metadata
    }

    /* == Persistence == */

    pub fn open(path: PathBuf, streams: Vec<StreamInfo>) -> Result<Self> {
        create_dir_all(&path)?;

        let metadata = SessionMetadata { streams };
        metadata.save(&path)?;

        Ok(Self {
            metadata,
            open_experiment: None,
            root_path: path,
        })
    }

    pub fn new_experiment(&mut self, name: String) -> Result<()> {
        let dir = self.root_path.join(Self::EXPERIMENTS_DIR);

        create_dir_all(&dir)?;

        let path = dir.join(Self::TEMP_EXPERIMENT_NAME);

        let mut file = File::create(path)?;
        let mut buf = ChunkedBytes::new();

        let header = ExperimentHeader::new(name, self.metadata.channels());
        header.encode(&mut buf);

        for chunk in buf.into_chunks() {
            file.write_all(&chunk)?;
        }

        self.open_experiment = Some((file, header.name));

        Ok(())
    }

    pub async fn append_run(&mut self, run: &Run) -> Result<()> {
        let Some((file, _)) = &mut self.open_experiment else {
            bail!("No experiment file open");
        };

        let mut buf = ChunkedBytes::new();
        run.encode(&mut buf);

        for chunk in buf.into_chunks() {
            file.write_all(&chunk)?;
        }

        file.flush()?;

        Ok(())
    }

    pub fn save_experiment(&mut self) -> Result<()> {
        let Some((file, name)) = self.open_experiment.take() else {
            bail!("No experiment file open");
        };

        drop(file);

        let id = self.next_experiment_number()?;
        let name = Self::experiment_name(id, &name);

        let dir = self.root_path.join(Self::EXPERIMENTS_DIR);

        create_dir_all(&dir)?;

        let from = dir.join(Self::TEMP_EXPERIMENT_NAME);
        let to = dir.join(&name);

        tracing::debug!("Saving experiment {name}");

        rename(from, to)?;

        Ok(())
    }

    pub fn list_experiments(&self) -> Result<Vec<(u32, PathBuf)>> {
        let dir = self.root_path.join(Self::EXPERIMENTS_DIR);
        let mut files = read_dir(dir)?;

        let mut experiments = Vec::new();

        while let Some(Ok(entry)) = files.next() {
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

    fn next_experiment_number(&self) -> Result<u32> {
        let path = self.root_path.join(Self::EXPERIMENTS_DIR);
        let mut dir = read_dir(path)?;
        let mut max_id = None;

        while let Some(Ok(entry)) = dir.next() {
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

    pub fn find(path: &Path) -> Option<Self> {
        let mut iter = path.ancestors();
        iter.next();

        for _ in 0..Self::FIND_LEVELS {
            match iter.next() {
                Some(path) => {
                    if let Ok(meta) = Self::load(path) {
                        return Some(meta);
                    }
                }

                None => break,
            }
        }

        None
    }

    pub fn load(path: &Path) -> Result<SessionMetadata> {
        let path = path.join(Self::FILE_NAME);
        let mut file = File::open(path)?;

        Ok(serde_json::from_reader(&mut file)?)
    }

    fn save(&self, path: &Path) -> io::Result<()> {
        let path = path.join(Self::FILE_NAME);
        let mut file = File::create(path)?;

        serde_json::to_writer(&mut file, self)?;
        file.flush()
    }

    fn channels(&self) -> Box<[u8]> {
        self.streams
            .iter()
            .map(|s| s.channels.len() as u8)
            .collect::<Vec<_>>()
            .into_boxed_slice()
    }
}
