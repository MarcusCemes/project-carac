use clap::Parser;
use eyre::Result;

use carac::cli::CliOpts;

fn main() -> Result<()> {
    let opts = CliOpts::parse();

    carac::init()?;
    carac::banner();

    carac::cli(opts)
}
