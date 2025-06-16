use clap::{Parser, Subcommand};
use eyre::Result;

use crate::{
    cli::{
        measure::{Measure, MeasureOpts},
        server::Server,
    },
    create_runtime,
};

mod common;
mod convert;
mod export;
mod extract;
mod measure;
mod plot;
mod server;
mod view;

#[derive(Parser)]
#[command(version, about)]
pub struct CliOpts {
    #[command(subcommand)]
    command: Option<CliCommand>,

    #[clap(flatten)]
    server: ServerOpts,
}

#[derive(Subcommand)]
pub enum CliCommand {
    Measure(MeasureOpts),
    Server(ServerOpts),
}

#[derive(Parser)]
pub struct ServerOpts {
    #[arg(short, long, default_value = "config.yaml")]
    pub config: String,

    #[arg(short, long, default_value_t = 8080)]
    pub port: u16,
}

#[derive(Subcommand)]
pub enum ServerCommand {
    Measure(MeasureOpts),
    Server(ServerOpts),
}

#[derive(Parser)]
#[command(version, about)]
pub struct KitOpts {
    #[command(subcommand)]
    command: KitCommand,
}

#[derive(Subcommand)]
pub enum KitCommand {
    Convert(convert::ConvertOpts),
    Export(export::ExportOpts),
    // ExportCalibrated(export_calibrated::ExportCalibratedOpts),
    Extract(extract::ExtractOpts),
    Plot(plot::PlotOpts),
    View(view::ViewOpts),
}

pub fn cli(opts: CliOpts) -> Result<()> {
    let rt = create_runtime();

    match opts.command {
        Some(command) => match command {
            CliCommand::Measure(opts) => rt.block_on(Measure::run(opts)),
            CliCommand::Server(opts) => rt.block_on(Server::launch(opts)),
        },
        None => rt.block_on(Server::launch(opts.server)),
    }
}

pub fn kit(opts: KitOpts) -> Result<()> {
    match opts.command {
        KitCommand::Convert(opts) => convert::segment(opts),
        KitCommand::Export(opts) => export::export(opts),
        // KitCommand::ExportCalibrated(opts) => export_calibrated::export_calibrated(opts),
        KitCommand::Extract(opts) => extract::extract(opts),
        KitCommand::Plot(opts) => plot::plot(opts),
        KitCommand::View(opts) => view::view(opts),
    }
}
