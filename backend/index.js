// backend/index.js
// Minimal MQTT -> Socket.IO bridge (hello world)

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

app.get("/health", (req, res) => {
  res.json({ ok: true, mqttUrl: MQTT_URL, topic: MQTT_TOPIC });
});

const httpServer = createServer(app);
const io = new Server(httpServer, {
  cors: { origin: "*" },
});

let lastMessage = null;

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

  lastMessage = msg;
  io.emit("telemetry", msg);
});

mqttClient.on("error", (err) => {
  console.error("MQTT error:", err.message);
});

httpServer.listen(PORT, () => {
  console.log(`Backend listening on http://localhost:${PORT}`);
});
