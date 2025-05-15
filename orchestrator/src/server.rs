use std::sync::Arc;

use axum::{
    Json, Router, extract,
    routing::{get, post},
};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;

use crate::data::orchestrator::{Instruction, Orchestrator};

pub struct AppState {
    pub orchestrator: Mutex<Orchestrator>,
}

pub fn create_router(orchestrator: Orchestrator) -> (Router, Arc<AppState>) {
    let state = Arc::new(AppState {
        orchestrator: Mutex::new(orchestrator),
    });

    let router = Router::new()
        .route("/execute", post(execute))
        .route("/save/experiment", post(save_experiment))
        .route("/save/run", post(save_run))
        .route("/status", get(status))
        .with_state(state.clone());

    (router, state)
}

/* === Routes === */

/* == Execute == */

#[derive(Deserialize)]
struct ExecutePayload {
    instructions: Vec<Instruction>,
}

#[derive(Serialize)]
#[serde(tag = "status")]
enum StandardResponse {
    Success,
    Error { message: String },
}

async fn execute(
    extract::State(state): extract::State<Arc<AppState>>,
    extract::Json(payload): extract::Json<ExecutePayload>,
) -> Json<StandardResponse> {
    let mut orchestrator = state.orchestrator.lock().await;

    let result = orchestrator
        .execute(payload.instructions)
        .await
        .map(|_| StandardResponse::Success)
        .unwrap_or_else(|e| StandardResponse::Error {
            message: e.to_string(),
        });

    Json(result)
}

/* == Save == */

async fn save_experiment(
    extract::State(state): extract::State<Arc<AppState>>,
) -> Json<StandardResponse> {
    let mut orchestrator = state.orchestrator.lock().await;

    let result = orchestrator
        .save_experiment()
        .await
        .map(|_| StandardResponse::Success)
        .unwrap_or_else(|r| StandardResponse::Error {
            message: r.to_string(),
        });

    Json(result)
}

async fn save_run(extract::State(state): extract::State<Arc<AppState>>) -> Json<StandardResponse> {
    let mut orchestrator = state.orchestrator.lock().await;

    let result = orchestrator
        .save_run()
        .await
        .map(|_| StandardResponse::Success)
        .unwrap_or_else(|r| StandardResponse::Error {
            message: r.to_string(),
        });

    Json(result)
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
