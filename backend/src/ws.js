// src/ws.js
// WebSocket layer (Socket.IO): emits last known state + receives drive commands.
// Comments in English as requested.

export function setupWebSocket({ io, mqttClient, TOPIC_CMD_DRIVE, state }) {
  io.on("connection", (socket) => {
    console.log("WS client connected:", socket.id);

    // On connect, send last known values (same behavior as current index.js)
    if (state.lastFast) socket.emit("telemetry:fast", state.lastFast);
    if (state.lastSlow) socket.emit("telemetry:slow", state.lastSlow);
    if (state.lastDriveState) socket.emit("drive:state", state.lastDriveState);
    if (state.lastDriveAck) socket.emit("drive:ack", state.lastDriveAck);

    // Frontend -> backend -> mqtt broker (robot/cmd/drive)
    socket.on("cmd:drive", (data) => {
      const L = Math.max(-255, Math.min(255, Number(data?.left) || 0));
      const R = Math.max(-255, Math.min(255, Number(data?.right) || 0));
      const payload = `${L},${R}`;

      mqttClient.publish(TOPIC_CMD_DRIVE, payload);

      socket.emit("cmd:drive:sent", {
        ok: true,
        left: L,
        right: R,
        payload,
        ts: Date.now(),
      });
    });

    socket.on("disconnect", () =>
      console.log("WS client disconnected:", socket.id)
    );
  });
}
