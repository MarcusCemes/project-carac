use eyre::Result;

fn main() -> Result<()> {
    orchestrator::init()?;
    orchestrator::cli::run()
}
