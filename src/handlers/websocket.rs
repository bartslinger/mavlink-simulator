use crate::simulator::Simulator;
use axum::extract::ws::{Message, WebSocket};
use axum::extract::{ConnectInfo, WebSocketUpgrade};
use axum::response::IntoResponse;
use bytes::BytesMut;
use futures_util::stream::SplitStream;
use futures_util::{SinkExt, Stream, StreamExt};
use mavlink::async_peek_reader::AsyncPeekReader;
use mavlink::{read_versioned_msg_async, write_versioned_msg, MavHeader};
use std::net::SocketAddr;
use std::task::Poll;
use tokio::io::AsyncRead;

pub async fn websocket_handler(
    ws: WebSocketUpgrade,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
) -> impl IntoResponse {
    tracing::info!("Incoming websocket connection");
    ws.on_upgrade(move |socket| ws_connection(socket, addr))
}

pub async fn ws_connection(socket: WebSocket, who: SocketAddr) {
    tracing::info!("ws connection from {}", who);
    let (mut tx, rx) = socket.split();

    let reader = WebSocketReader::new(rx);
    let mut peek_reader = AsyncPeekReader::new(reader);

    let (_uplink_tx, uplink_rx) = tokio::sync::mpsc::channel(10);
    let (downlink_tx, mut downlink_rx) = tokio::sync::mpsc::channel(10);

    let simulator = Simulator::new();

    let receive_task = async move {
        while let Ok((header, message)) = read_versioned_msg_async::<
            mavlink::ardupilotmega::MavMessage,
            _,
        >(&mut peek_reader, mavlink::MavlinkVersion::V2)
        .await
        {
            tracing::info!("Incoming via websocket: {:?} {:?}", header, message);
            tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        }
    };

    let send_task = async move {
        let mut header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };
        while let Some(message) = downlink_rx.recv().await {
            // encode
            header.sequence = header.sequence.wrapping_add(1);
            let mut buf = Vec::new();
            let write_result =
                write_versioned_msg(&mut buf, mavlink::MavlinkVersion::V2, header, &message);
            if let Err(e) = write_result {
                tracing::error!("Error writing message: {:?}", e);
                continue;
            }
            let send_ws_result = tx.send(Message::Binary(buf.into())).await;
            if let Err(e) = send_ws_result {
                tracing::error!("Error sending message: {:?}", e);
            }
        }
    };

    tokio::select! {
        _ = receive_task => {
            tracing::info!("Receive task completed");
        }
        _ = send_task => {
            tracing::info!("Send task completed");
        }
        _ = simulator.run(downlink_tx, uplink_rx) => {
            tracing::info!("Simulator task completed");
        }
    }
}

pub struct WebSocketReader {
    stream: SplitStream<WebSocket>,
    buffer: BytesMut,
}

impl WebSocketReader {
    pub fn new(stream: SplitStream<WebSocket>) -> Self {
        Self {
            stream,
            buffer: BytesMut::new(),
        }
    }
}

impl AsyncRead for WebSocketReader {
    fn poll_read(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &mut tokio::io::ReadBuf<'_>,
    ) -> Poll<std::io::Result<()>> {
        while self.buffer.is_empty() {
            match futures_util::ready!(std::pin::Pin::new(&mut self.stream).poll_next(cx)) {
                Some(Ok(Message::Binary(b))) => self.buffer.extend_from_slice(&b),
                Some(Ok(Message::Text(s))) => self.buffer.extend_from_slice(s.as_bytes()),
                Some(Ok(_)) => continue, // Ignore Ping/Pong/Close
                Some(Err(e)) => {
                    return Poll::Ready(Err(std::io::Error::new(std::io::ErrorKind::Other, e)))
                }
                None => return Poll::Ready(Ok(())), // EOF
            }
        }

        let to_copy = std::cmp::min(buf.remaining(), self.buffer.len());
        buf.put_slice(&self.buffer.split_to(to_copy));
        Poll::Ready(Ok(()))
    }
}
