use std::path::PathBuf;

use clap::ValueEnum;

enum KitSource {
    Experiment { input: PathBuf, output: PathBuf },
    Session { input: PathBuf, output: String },
}

struct FilterOpts {
    cutoff: Option<f32>,
    order: Option<u8>,
}

#[derive(Clone, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum OutputFormat {
    Csv,
    Parquet,
}

impl OutputFormat {
    pub fn extension(&self) -> &'static str {
        match self {
            OutputFormat::Csv => "csv",
            OutputFormat::Parquet => "parquet",
        }
    }
}
