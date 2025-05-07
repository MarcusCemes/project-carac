use std::path::PathBuf;

use eyre::{bail, Context, Result};
use tokio::{
    fs,
    io::{self, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt},
};

use crate::{
    data::run::{Run, StreamInfo},
    misc::buf::{ReadExt, WriteExt},
};

/* === Definitions === */

pub struct ExperimentManager {
    experiment_counter: u32,
    path: PathBuf,
}

pub struct Experiment {
    pub metadata: ExperimentMetadata,
    pub runs: Vec<Run>,
}

pub struct ExperimentMetadata {
    pub name: Option<String>,
    pub timestamp_ms: i64,
    pub streams: Vec<StreamInfo>,
}

/* === Implementations === */

impl ExperimentManager {
    pub async fn create(path: PathBuf) -> Result<Self> {
        if path.exists() {
            bail!("Path already exists: {}", path.display());
        }

        Ok(Self {
            experiment_counter: 0,
            path,
        })
    }

    pub async fn save_experiment(&mut self, experiment: &Experiment) -> Result<()> {
        let experiment_path = self
            .path
            .join(format!("experiment_{}", self.experiment_counter));

        let file = fs::File::create(&experiment_path)
            .await
            .wrap_err("Failed to open experiment file")?;

        let mut writer = io::BufWriter::new(file);

        experiment.write(&mut writer).await?;

        self.experiment_counter += 1;

        Ok(())
    }
}

impl Experiment {
    pub fn new(metadata: ExperimentMetadata, runs: Vec<Run>) -> Self {
        Self { metadata, runs }
    }

    pub async fn read<R: AsyncRead + Unpin>(r: &mut R) -> io::Result<Self> {
        let metadata = ExperimentMetadata::read(r).await?;

        let n_runs = r.read_u16().await? as usize;
        let mut runs = Vec::with_capacity(n_runs);

        for _ in 0..n_runs {
            let run = Run::read(r, &metadata.streams).await?;
            runs.push(run);
        }

        Ok(Experiment { metadata, runs })
    }

    pub async fn write<W: AsyncWrite + Unpin>(&self, w: &mut W) -> io::Result<()> {
        self.metadata.write(w).await?;

        w.write_u16(self.runs.len() as u16).await?;

        for run in &self.runs {
            run.write(w).await?;
        }

        Ok(())
    }
}

impl ExperimentMetadata {
    pub fn new(name: Option<String>, streams: Vec<StreamInfo>) -> Self {
        Self {
            name,
            timestamp_ms: chrono::Utc::now().timestamp_millis(),
            streams,
        }
    }

    pub async fn read<R: AsyncRead + Unpin>(r: &mut R) -> io::Result<Self> {
        let timestamp_ms = r.read_i64().await?;

        let name = match r.read_u8().await? {
            0 => None,
            _ => Some(r.read_string().await?),
        };

        let n_streams = r.read_u8().await? as usize;
        let mut streams = Vec::with_capacity(n_streams);

        for _ in 0..n_streams {
            let stream = StreamInfo::read(r).await?;

            streams.push(stream);
        }

        Ok(Self {
            name,
            timestamp_ms,
            streams,
        })
    }

    pub async fn write<W: AsyncWrite + Unpin>(&self, w: &mut W) -> io::Result<()> {
        w.write_i64(self.timestamp_ms).await?;

        match self.name {
            Some(ref name) => {
                w.write_u8(1).await?;
                w.write_string(name).await?
            }

            None => w.write_u8(0).await?,
        }

        w.write_u8(self.streams.len() as u8).await?;

        for stream in &self.streams {
            stream.write(w).await?;
        }

        Ok(())
    }
}
