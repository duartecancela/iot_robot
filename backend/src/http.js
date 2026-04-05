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
  mqttClient,
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
        allowed: [
          "robot/telemetry/*",
          TOPIC_STATE_DRIVE,
          TOPIC_CMD_DRIVE_ACK,
          "robot/tracking/command",
        ],
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
      tracking: state.tracking || {
        robot_is_moving: false,
        stop_timestamp: null,
        tracking_allowed: true,
        tracking_active: false,
      },
      lastMessage: state.lastMessage,
    });
  });

  // POST /tracking
  app.post("/tracking", (req, res) => {
    const { tracking_allowed } = req.body || {};

    if (typeof tracking_allowed !== "boolean") {
      return res.status(400).json({
        ok: false,
        error: "tracking_allowed must be a boolean",
      });
    }

    if (!mqttClient || !mqttClient.connected) {
      return res.status(503).json({
        ok: false,
        error: "MQTT client is not connected",
      });
    }

    const payload = JSON.stringify({ tracking_allowed });

    mqttClient.publish("robot/tracking/command", payload, (err) => {
      if (err) {
        return res.status(500).json({
          ok: false,
          error: err.message,
        });
      }

      return res.json({
        ok: true,
        published: {
          topic: "robot/tracking/command",
          payload: { tracking_allowed },
        },
      });
    });
  });

  return app;
}