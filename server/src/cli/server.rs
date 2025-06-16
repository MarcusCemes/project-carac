use std::{net::Ipv4Addr, sync::Arc};

use axum::{
    Json, Router,
    extract::{self, DefaultBodyLimit},
    http::StatusCode,
    response::IntoResponse,
    routing::{get, post},
    serve,
};
use eyre::Result;
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;

use crate::{
    cli::ServerOpts,
    config::Config,
    data::orchestrator::{Instruction, Orchestrator},
    hardware::HardwareContext,
};

pub struct Server;

impl Server {
    pub async fn launch(opts: ServerOpts) -> Result<()> {
        let config = Config::load(&opts.config).await?;
        tracing::info!("{}", config.hardware);

        let context = HardwareContext::builder().build(&config.hardware).await?;

        let orchestrator = Orchestrator::try_new(config, context).await?;
        let (app, state) = create_router(orchestrator);

        let socket = TcpListener::bind((Ipv4Addr::UNSPECIFIED, opts.port)).await?;

        state.orchestrator.start().await;

        tracing::info!("Listening on http://0.0.0.0:{}", opts.port);
        serve(socket, app).await?;

        state.orchestrator.stop().await;

        Ok(())
    }
}

/* === Router === */

pub struct AppState {
    pub orchestrator: Orchestrator,
}

pub fn create_router(orchestrator: Orchestrator) -> (Router, Arc<AppState>) {
    let state = Arc::new(AppState { orchestrator });

    let router = Router::new()
        .route("/execute", post(execute))
        .route("/record", post(record))
        .layer(DefaultBodyLimit::disable())
        .route("/new-experiment", post(new_experiment))
        .route("/save-experiment", post(save_experiment))
        .route("/start-recording", post(start_recording))
        .route("/stop-recording", post(stop_recording))
        .route("/status", get(status))
        .route("/progress", get(progress))
        .with_state(state.clone());

    (router, state)
}

/* == Execute == */

#[derive(Deserialize)]
struct ExecutePayload {
    instructions: Vec<Instruction>,
}

async fn execute(
    extract::State(state): extract::State<Arc<AppState>>,
    extract::Json(payload): extract::Json<ExecutePayload>,
) -> StandardResponse {
    state
        .orchestrator
        .execute(payload.instructions)
        .await
        .map(|_| ())
        .into()
}

async fn record(
    extract::State(state): extract::State<Arc<AppState>>,
    extract::Json(payload): extract::Json<ExecutePayload>,
) -> StandardResponse {
    state
        .orchestrator
        .record(payload.instructions)
        .await
        .map(|_| ())
        .into()
}

/* == Experiments == */

#[derive(Deserialize)]
struct NewExperimentPayload {
    name: String,
}

async fn new_experiment(
    extract::State(state): extract::State<Arc<AppState>>,
    extract::Json(payload): extract::Json<NewExperimentPayload>,
) -> StandardResponse {
    state.orchestrator.new_experiment(payload.name).await.into()
}

async fn save_experiment(extract::State(state): extract::State<Arc<AppState>>) -> StandardResponse {
    state.orchestrator.save_experiment().await.into()
}

async fn start_recording(extract::State(state): extract::State<Arc<AppState>>) -> StandardResponse {
    state.orchestrator.start_recording().await;
    Ok(()).into()
}

async fn stop_recording(extract::State(state): extract::State<Arc<AppState>>) -> StandardResponse {
    state.orchestrator.stop_recording().await.map(|_| ()).into()
}

#[derive(Serialize)]
struct Progress {
    pub progress: f32,
}

async fn progress(extract::State(state): extract::State<Arc<AppState>>) -> Json<Progress> {
    let progress = state.orchestrator.get_progress().unwrap_or(0.0);
    Json(Progress { progress })
}

/* == Types == */

#[derive(Serialize)]
#[serde(tag = "status")]
enum StandardResponse<T = ()> {
    Success { data: T },
    Error { message: String },
}

impl<T: Serialize> From<Result<T>> for StandardResponse<T> {
    fn from(result: Result<T>) -> Self {
        match result {
            Ok(data) => StandardResponse::Success { data },
            Err(error) => StandardResponse::Error {
                message: error.to_string(),
            },
        }
    }
}

impl<T: Serialize> IntoResponse for StandardResponse<T> {
    fn into_response(self) -> axum::response::Response {
        match self {
            StandardResponse::Success { .. } => (StatusCode::OK, Json(self)),
            StandardResponse::Error { .. } => (StatusCode::BAD_REQUEST, Json(self)),
        }
        .into_response()
    }
}

/* == Status == */

async fn status() -> &'static str {
    "Healthy"
}
