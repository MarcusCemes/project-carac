use std::sync::Arc;

use axum::{
    Json, Router, extract,
    routing::{get, post},
};
use eyre::Result;
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;

use crate::data::orchestrator::{Instruction, Orchestrator};

struct AppState {
    orchestrator: Mutex<Orchestrator>,
}

pub fn create_router(orchestrator: Orchestrator) -> Router {
    Router::new()
        .route("/execute", post(execute))
        .route("/finish/experiment", post(finish_experiment))
        .route("/finish/run", post(finish_run))
        .route("/start", post(start))
        .route("/status", get(status))
        .route("/stop", post(stop))
        .with_state(Arc::new(AppState {
            orchestrator: Mutex::new(orchestrator),
        }))
}

/* === Routes === */

/* == Execute == */

#[derive(Deserialize)]
struct ExecutePayload {
    instructions: Vec<Instruction>,
}

#[derive(Serialize)]
enum ExecuteResponse {
    Success,
    Error { message: String },
}

async fn execute(
    extract::State(state): extract::State<Arc<AppState>>,
    extract::Json(payload): extract::Json<ExecutePayload>,
) -> Json<ExecuteResponse> {
    let mut orchestrator = state.orchestrator.lock().await;

    for instruction in payload.instructions {
        if let Err(error) = orchestrator.execute(instruction).await {
            return Json(ExecuteResponse::Error {
                message: error.to_string(),
            });
        }
    }

    Json(ExecuteResponse::Success)
}

/* == Finish ==  */

async fn finish_experiment(extract::State(state): extract::State<Arc<AppState>>) {
    state.orchestrator.lock().await.finish_experiment().await;
}

async fn finish_run(extract::State(state): extract::State<Arc<AppState>>) -> Result<(), String> {
    state
        .orchestrator
        .lock()
        .await
        .finish_run()
        .await
        .map_err(|r| r.to_string())
}

/* == Start == */

async fn start(extract::State(state): extract::State<Arc<AppState>>) {
    state.orchestrator.lock().await.start().await;
}

/* == Status == */

async fn status() -> &'static str {
    "Healthy"
}

/* == Stop == */

async fn stop(extract::State(state): extract::State<Arc<AppState>>) {
    state.orchestrator.lock().await.stop().await;
}
