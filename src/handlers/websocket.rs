use axum::extract::ws::WebSocket;
use axum::extract::{ConnectInfo, WebSocketUpgrade};
use axum::response::IntoResponse;
use futures_util::StreamExt;
use std::net::SocketAddr;

pub async fn websocket_handler(
    ws: WebSocketUpgrade,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
) -> impl IntoResponse {
    tracing::info!("Incoming websocket connection");
    ws.on_upgrade(move |socket| ws_connection(socket, addr))
}

pub async fn ws_connection(socket: WebSocket, who: SocketAddr) {
    tracing::info!("ws connection from {}", who);
    let (_tx, mut rx) = socket.split();

    let receive_task = async move {
        while let Some(Ok(msg)) = rx.next().await {
            match msg {
                axum::extract::ws::Message::Text(text) => {
                    tracing::info!("Received text: {}", text);
                }
                axum::extract::ws::Message::Binary(_) => {
                    tracing::info!("Received binary");
                }
                axum::extract::ws::Message::Close(_) => {
                    tracing::info!("Connection closed");
                    break;
                }
                _ => {}
            }
        }
    };

    tokio::select! {
        _ = receive_task => {
            tracing::info!("Receive task completed");
        }
    }
}
