use clap::{Parser, Subcommand};
use eyre::Result;

mod convert;
mod export;
mod measure;
mod plot;
mod run;
mod server;
mod view;

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    Convert(convert::ConvertOpts),
    Export(export::ExportOpts),
    Measure(measure::MeasureOpts),
    Plot(plot::PlotOpts),
    View(view::ViewOpts),

    Run {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },

    Server {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
        #[arg(short, long, default_value_t = 8080)]
        port: u16,
    },
}

#[derive(Subcommand)]
enum ConfigCommand {
    Test {
        #[arg(short, long, default_value = "config.yaml")]
        config: String,
    },
}

pub fn run() -> Result<()> {
    execute_command(Cli::parse().command)
}

#[tokio::main(worker_threads = 4)]
pub async fn execute_command(command: Command) -> Result<()> {
    match command {
        Command::Convert(opts) => convert::segment(opts).await,
        Command::Export(opts) => export::export(opts).await,
        Command::Measure(opts) => measure::Measure::run(opts).await,
        Command::Plot(opts) => plot::plot(opts).await,
        Command::Run { config } => run::launch(&config).await,
        Command::Server { config, port } => server::launch(&config, port).await,
        Command::View(opts) => view::view(opts).await,
    }
}
