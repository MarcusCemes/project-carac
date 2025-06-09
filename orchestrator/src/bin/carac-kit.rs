use clap::Parser;
use eyre::Result;

use carac::cli::KitOpts;

fn main() -> Result<()> {
    let opts = KitOpts::parse();

    carac::init()?;
    carac::banner();

    carac::kit(opts)
}
