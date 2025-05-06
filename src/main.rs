use anyhow::Context;
use axum::routing::{any, get};
use std::net::SocketAddr;

mod fixed_wing;
mod handlers;
mod simulator;

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    // Initialize logging
    tracing_subscriber::fmt::init();
    tracing::info!("Hello, world!");

    // Fine as long as this is not publicly exposed
    let cors_layer = tower_http::cors::CorsLayer::new()
        .allow_origin(tower_http::cors::Any) // Allow any origin
        .allow_methods(tower_http::cors::Any) // Allow any HTTP method
        .allow_headers(tower_http::cors::Any); // Allow any headers

    let app = axum::Router::new()
        .route("/", get(|| async { "Hello, world!" }))
        .route("/ws", any(handlers::websocket::websocket_handler))
        .layer(cors_layer)
        .into_make_service_with_connect_info::<SocketAddr>();

    let listener = tokio::net::TcpListener::bind("[::]:3000").await?;

    axum::serve(listener, app).await.context("Server failed")
}
