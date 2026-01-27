// backend/index.js
// Minimal MQTT -> Socket.IO bridge (hello world) + aggregated telemetry state

import express from "express";
import cors from "cors";
import { createServer } from "http";
import { Server } from "socket.io";
import mqtt from "mqtt";
import dotenv from "dotenv";

dotenv.config();

const PORT = Number(process.env.PORT || 3001);
const MQTT_URL = process.env.MQTT_URL || "mqtt://127.0.0.1:1883";
const MQTT_TOPIC = process.env.MQTT_TOPIC || "robot/telemetry/#";

const app = express();
app.use(cors());
app.use(express.json());

// --- Health / debug endpoints ---
app.get("/health", (req, res) => {
  res.json({ ok: true, mqttUrl: MQTT_URL, topic: MQTT_TOPIC });
});

// Last message (any topic)
let lastMessage = null;

// Aggregated state (last known values per channel/topic)
let lastFast = null; // robot/telemetry/fast
let lastSlow = null; // robot/telemetry/slow

app.get("/telemetry/last", (req, res) => {
  res.json(lastMessage ?? { ok: false, error: "No telemetry received yet" });
});

app.get("/telemetry/state", (req, res) => {
  res.json({
    ok: true,
    mqtt: { url: MQTT_URL, topic: MQTT_TOPIC },
    ts: Date.now(),
    fast: lastFast, // null until first fast message arrives
    slow: lastSlow, // null until first slow message arrives
  });
});

const httpServer = createServer(app);
const io = new Server(httpServer, {
  cors: { origin: "*" },
});

io.on("connection", (socket) => {
  console.log("Web client connected:", socket.id);

  // Send last known message on connect (nice for hello world)
  if (lastMessage) socket.emit("telemetry", lastMessage);

  socket.on("disconnect", () => {
    console.log("Web client disconnected:", socket.id);
  });
});

console.log("Connecting MQTT:", MQTT_URL);
const mqttClient = mqtt.connect(MQTT_URL);

mqttClient.on("connect", () => {
  console.log("MQTT connected. Subscribing:", MQTT_TOPIC);
  mqttClient.subscribe(MQTT_TOPIC, { qos: 0 });
});

mqttClient.on("message", (topic, payload) => {
  const raw = payload.toString();
  console.log("[MQTT IN]", topic, raw);

  const msg = {
    topic,
    raw,
    ts: Date.now(),
  };

  // Try parse JSON, but keep raw always
  try {
    msg.json = JSON.parse(raw);
  } catch (e) {
    msg.json = null;
  }

  // Update "last message"
  lastMessage = msg;

  // Update aggregated state by topic
  if (topic === "robot/telemetry/fast") {
    lastFast = msg;
  } else if (topic === "robot/telemetry/slow") {
    lastSlow = msg;
  }

  // Push to any connected web clients (future frontend)
  io.emit("telemetry", msg);
});

mqttClient.on("error", (err) => {
  console.error("MQTT error:", err.message);
});

httpServer.listen(PORT, () => {
  console.log(`Backend listening on http://localhost:${PORT}`);
});
