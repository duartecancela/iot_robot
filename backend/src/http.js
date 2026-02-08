// src/http.js
// HTTP layer (Express): routes and JSON responses.
// Comments in English as requested.

import express from "express";
import cors from "cors";

export function createHttpApp({
  MQTT_URL,
  MQTT_TOPIC,
  TOPIC_STATE_DRIVE,
  TOPIC_CMD_DRIVE_ACK,
  state,
}) {
  const app = express();
  app.use(cors());
  app.use(express.json());

  // GET /health
  app.get("/health", (req, res) => {
    res.json({
      ok: true,
      mqtt: {
        url: MQTT_URL,
        wildcard: MQTT_TOPIC,
        allowed: ["robot/telemetry/*", TOPIC_STATE_DRIVE, TOPIC_CMD_DRIVE_ACK],
        dropped: ["robot/cmd/* (except robot/cmd/drive/ack)"],
      },
    });
  });

  // GET /telemetry/state
  app.get("/telemetry/state", (req, res) => {
    res.json({
      ok: true,
      ts: Date.now(),
      counters: state.counters,
      fast: state.lastFast,
      slow: state.lastSlow,
      drive: {
        state: state.lastDriveState,
        ack: state.lastDriveAck,
      },
      lastMessage: state.lastMessage,
    });
  });

  return app;
}
