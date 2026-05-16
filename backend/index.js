import dotenv from "dotenv";
dotenv.config();

import http from "http";
import { Server } from "socket.io";
import mqtt from "mqtt";

import { createHttpApp } from "./src/http.js";
import { setupWebSocket } from "./src/ws.js";
import { setupMqtt } from "./src/mqtt.js";
import { connectMongo } from "./src/db.js";

// --------------------
// Config (same defaults)
// --------------------
const PORT = Number(process.env.PORT || 3001);

const MQTT_URL =
  process.env.MQTT_URL ||
  `mqtt://${process.env.MQTT_HOST || "127.0.0.1"}:${process.env.MQTT_PORT || "1883"}`;

const MQTT_TOPIC = process.env.MQTT_TOPIC || "robot/#";

const TOPIC_STATE_DRIVE =
  process.env.MQTT_TOPIC_STATE_DRIVE || "robot/state/drive";
const TOPIC_CMD_DRIVE_ACK =
  process.env.MQTT_TOPIC_CMD_DRIVE_ACK || "robot/cmd/drive/ack";

const TOPIC_CMD_DRIVE =
  process.env.MQTT_TOPIC_CMD_DRIVE || "robot/cmd/drive";

// --------------------
// Shared State
// --------------------
const state = {
  lastFast: null,
  lastSlow: null,
  lastDriveState: null,
  lastDriveAck: null,
  lastMessage: null,
  logging: {
    enabled: false,
    startedAt: null,
  },
  counters: {
    total: 0,
    allowed: 0,
    dropped: 0,
    byTopic: {},
  },
};

// --------------------
// MongoDB
// --------------------
await connectMongo();

// --------------------
// MQTT client
// --------------------
const mqttClient = mqtt.connect(MQTT_URL, {
  reconnectPeriod: 2000,
  connectTimeout: 5000,
});

// --------------------
// HTTP + Socket.IO
// --------------------
const app = createHttpApp({
  MQTT_URL,
  MQTT_TOPIC,
  TOPIC_STATE_DRIVE,
  TOPIC_CMD_DRIVE_ACK,
  state,
  mqttClient,
});

const server = http.createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

// --------------------
// Wire MQTT behavior
// --------------------
setupMqtt({
  mqttClient,
  MQTT_URL,
  MQTT_TOPIC,
  TOPIC_STATE_DRIVE,
  TOPIC_CMD_DRIVE_ACK,
  state,
  io,
});

// --------------------
// Wire WS behavior
// --------------------
setupWebSocket({
  io,
  mqttClient,
  TOPIC_CMD_DRIVE,
  state,
});

// --------------------
server.listen(PORT, () => {
  console.log(`Backend listening on http://localhost:${PORT}`);
});