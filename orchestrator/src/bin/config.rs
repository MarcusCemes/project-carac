use clap::{Parser, arg};
use eyre::Result;

use drone_lab::config::Config;

#[derive(Parser)]
struct Opts {
    #[arg(short, long, default_value = "config.yaml")]
    path: String,
}

#[tokio::main(flavor = "current_thread")]
pub async fn main() -> Result<()> {
    drone_lab::init()?;

    let opts = Opts::parse();
    let config = Config::load(&opts.path).await?;
    println!("{config:#?}");
    Ok(())
}
