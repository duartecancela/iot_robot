import dotenv from "dotenv";
dotenv.config();

import express from "express";
import cors from "cors";
import http from "http";
import { Server } from "socket.io";
import mqtt from "mqtt";

// --------------------
// Config
// --------------------
const PORT = Number(process.env.PORT || 3001);

const MQTT_URL =
  process.env.MQTT_URL ||
  `mqtt://${process.env.MQTT_HOST || "127.0.0.1"}:${process.env.MQTT_PORT || "1883"}`;

const MQTT_TOPIC = process.env.MQTT_TOPIC || "robot/#";

// Confirmed / applied topics
const TOPIC_STATE_DRIVE =
  process.env.MQTT_TOPIC_STATE_DRIVE || "robot/state/drive";
const TOPIC_CMD_DRIVE_ACK =
  process.env.MQTT_TOPIC_CMD_DRIVE_ACK || "robot/cmd/drive/ack";

// Command topic (frontend → broker)
const TOPIC_CMD_DRIVE =
  process.env.MQTT_TOPIC_CMD_DRIVE || "robot/cmd/drive";

// --------------------
// Helpers
// --------------------
function isAllowedIncomingTopic(topic) {
  if (!topic) return false;
  if (topic.startsWith("robot/telemetry/")) return true;
  if (topic === TOPIC_STATE_DRIVE) return true;
  if (topic === TOPIC_CMD_DRIVE_ACK) return true;
  return false;
}

// --------------------
// Express + Socket.IO
// --------------------
const app = express();
app.use(cors());
app.use(express.json());

const server = http.createServer(app);
const io = new Server(server, { cors: { origin: "*" } });

// --------------------
// State
// --------------------
let lastFast = null;
let lastSlow = null;
let lastDriveState = null;
let lastDriveAck = null;
let lastMessage = null;

const counters = {
  total: 0,
  allowed: 0,
  dropped: 0,
  byTopic: {},
};

function bumpTopic(topic) {
  counters.byTopic[topic] = (counters.byTopic[topic] || 0) + 1;
}

// --------------------
// HTTP endpoints
// --------------------
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
      ],
      dropped: ["robot/cmd/* (except robot/cmd/drive/ack)"],
    },
  });
});

app.get("/telemetry/state", (req, res) => {
  res.json({
    ok: true,
    ts: Date.now(),
    counters,
    fast: lastFast,
    slow: lastSlow,
    drive: {
      state: lastDriveState,
      ack: lastDriveAck,
    },
    lastMessage,
  });
});

// --------------------
// WebSocket
// --------------------
io.on("connection", (socket) => {
  console.log("WS client connected:", socket.id);

  if (lastFast) socket.emit("telemetry:fast", lastFast);
  if (lastSlow) socket.emit("telemetry:slow", lastSlow);
  if (lastDriveState) socket.emit("drive:state", lastDriveState);
  if (lastDriveAck) socket.emit("drive:ack", lastDriveAck);

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

// --------------------
// MQTT
// --------------------
console.log("MQTT_URL =", MQTT_URL);
console.log("MQTT_TOPIC =", MQTT_TOPIC);

const mqttClient = mqtt.connect(MQTT_URL, {
  reconnectPeriod: 2000,
  connectTimeout: 5000,
});

mqttClient.on("connect", () => {
  console.log("MQTT: connected");

  mqttClient.subscribe(MQTT_TOPIC, (err) => {
    if (err) console.log("MQTT subscribe error:", err.message);
    else console.log("MQTT subscribed:", MQTT_TOPIC);
  });

  mqttClient.subscribe(
    [TOPIC_STATE_DRIVE, TOPIC_CMD_DRIVE_ACK],
    (err) => {
      if (err)
        console.log("MQTT subscribe (required) error:", err.message);
      else
        console.log(
          "MQTT subscribed (required):",
          TOPIC_STATE_DRIVE,
          TOPIC_CMD_DRIVE_ACK
        );
    }
  );
});

mqttClient.on("reconnect", () => console.log("MQTT: reconnect"));
mqttClient.on("offline", () => console.log("MQTT: offline"));
mqttClient.on("close", () => console.log("MQTT: close"));
mqttClient.on("error", (err) =>
  console.log("MQTT error:", err.message)
);

mqttClient.on("message", (topic, payload) => {
  counters.total++;
  bumpTopic(topic);

  const raw = payload.toString();
  const allowed = isAllowedIncomingTopic(topic);

  if (allowed) counters.allowed++;
  else counters.dropped++;

  console.log(`${allowed ? "ALLOW" : "DROP "} MQTT -> ${topic} ${raw}`);

  if (!allowed) return;

  const msg = { topic, raw, ts: Date.now() };
  try {
    msg.json = JSON.parse(raw);
  } catch {
    msg.json = null;
  }

  lastMessage = msg;

  if (topic === "robot/telemetry/fast") {
    lastFast = msg;
    io.emit("telemetry:fast", msg);
  } else if (topic === "robot/telemetry/slow") {
    lastSlow = msg;
    io.emit("telemetry:slow", msg);
  } else if (topic === TOPIC_STATE_DRIVE) {
    lastDriveState = msg;
    io.emit("drive:state", msg);
  } else if (topic === TOPIC_CMD_DRIVE_ACK) {
    lastDriveAck = msg;
    io.emit("drive:ack", msg);
  }
});

// --------------------
server.listen(PORT, () => {
  console.log(`Backend listening on http://localhost:${PORT}`);
});
