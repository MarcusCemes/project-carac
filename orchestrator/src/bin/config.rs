use clap::{arg, Parser};
use eyre::Result;

use orchestrator::config::Config;

#[derive(Parser)]
struct Opts {
    #[arg(short, long, default_value = "config.yaml")]
    path: String,
}

#[tokio::main(flavor = "current_thread")]
pub async fn main() -> Result<()> {
    orchestrator::init()?;

    let opts = Opts::parse();
    let config = Config::load(&opts.path).await?;
    println!("{config:#?}");
    Ok(())
}
