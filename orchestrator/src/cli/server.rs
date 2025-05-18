use std::{net::Ipv4Addr, sync::Arc, time::Duration};

use axum::{
    Json, Router, extract,
    http::StatusCode,
    response::IntoResponse,
    routing::{get, post},
    serve,
};
use eyre::Result;
use serde::{Deserialize, Serialize};
use tokio::{net::TcpListener, sync::Mutex};

use crate::{
    config::Config,
    data::orchestrator::{Instruction, Orchestrator},
    hardware::HardwareContext,
};

const HARDWARE_TIMEOUT: Duration = Duration::from_secs(1);

pub async fn launch(config_path: &str, port: u16) -> Result<()> {
    let config = Config::load(config_path).await?;
    tracing::info!("{}", config.hardware);

    let context = HardwareContext::builder().build(&config.hardware).await?;

    let orchestrator = Orchestrator::create(config, context).await?;
    let (app, state) = create_router(orchestrator);

    let socket = TcpListener::bind((Ipv4Addr::UNSPECIFIED, port)).await?;

    state.orchestrator.lock().await.start().await;

    tracing::info!("Listening on http://0.0.0.0:{port}");
    serve(socket, app).await?;

    state.orchestrator.lock().await.stop().await;

    Ok(())
}

/* === Router === */

pub struct AppState {
    pub orchestrator: Mutex<Orchestrator>,
}

pub fn create_router(orchestrator: Orchestrator) -> (Router, Arc<AppState>) {
    let state = Arc::new(AppState {
        orchestrator: Mutex::new(orchestrator),
    });

    let router = Router::new()
        .route("/execute", post(execute))
        .route("/record", post(record))
        .route("/new_experiment", post(new_experiment))
        .route("/save_experiment", post(save_experiment))
        .route("/status", get(status))
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
    let mut orchestrator = state.orchestrator.lock().await;

    orchestrator
        .execute(payload.instructions)
        .await
        .map(|_| ())
        .into()
}

async fn record(
    extract::State(state): extract::State<Arc<AppState>>,
    extract::Json(payload): extract::Json<ExecutePayload>,
) -> StandardResponse {
    let mut orchestrator = state.orchestrator.lock().await;

    orchestrator
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
    let mut orchestrator = state.orchestrator.lock().await;
    orchestrator.new_experiment(payload.name).await.into()
}

async fn save_experiment(extract::State(state): extract::State<Arc<AppState>>) -> StandardResponse {
    let mut orchestrator = state.orchestrator.lock().await;
    orchestrator.save_experiment().await.into()
}

async fn save_run(extract::State(state): extract::State<Arc<AppState>>) -> StandardResponse {
    let mut orchestrator = state.orchestrator.lock().await;
    orchestrator.save_run().await.map(|_| ()).into()
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
