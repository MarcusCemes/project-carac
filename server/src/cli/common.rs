use clap::ValueEnum;

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
