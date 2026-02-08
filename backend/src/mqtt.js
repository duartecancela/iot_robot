// src/mqtt.js
// MQTT layer: connect/subscribe + message routing + allowlist.
// Comments in English as requested.

export function isAllowedIncomingTopicFactory({
  TOPIC_STATE_DRIVE,
  TOPIC_CMD_DRIVE_ACK,
}) {
  return function isAllowedIncomingTopic(topic) {
    if (!topic) return false;
    if (topic.startsWith("robot/telemetry/")) return true;
    if (topic === TOPIC_STATE_DRIVE) return true;
    if (topic === TOPIC_CMD_DRIVE_ACK) return true;
    return false;
  };
}

export function bumpTopic(counters, topic) {
  counters.byTopic[topic] = (counters.byTopic[topic] || 0) + 1;
}

export function setupMqtt({
  mqttClient,
  MQTT_URL,
  MQTT_TOPIC,
  TOPIC_STATE_DRIVE,
  TOPIC_CMD_DRIVE_ACK,
  state,
  io,
}) {
  console.log("MQTT_URL =", MQTT_URL);
  console.log("MQTT_TOPIC =", MQTT_TOPIC);

  const isAllowedIncomingTopic = isAllowedIncomingTopicFactory({
    TOPIC_STATE_DRIVE,
    TOPIC_CMD_DRIVE_ACK,
  });

  mqttClient.on("connect", () => {
    console.log("MQTT: connected");

    mqttClient.subscribe(MQTT_TOPIC, (err) => {
      if (err) console.log("MQTT subscribe error:", err.message);
      else console.log("MQTT subscribed:", MQTT_TOPIC);
    });

    // Keep required subscriptions (same as current code)
    mqttClient.subscribe([TOPIC_STATE_DRIVE, TOPIC_CMD_DRIVE_ACK], (err) => {
      if (err) console.log("MQTT subscribe (required) error:", err.message);
      else
        console.log(
          "MQTT subscribed (required):",
          TOPIC_STATE_DRIVE,
          TOPIC_CMD_DRIVE_ACK
        );
    });
  });

  mqttClient.on("reconnect", () => console.log("MQTT: reconnect"));
  mqttClient.on("offline", () => console.log("MQTT: offline"));
  mqttClient.on("close", () => console.log("MQTT: close"));
  mqttClient.on("error", (err) => console.log("MQTT error:", err.message));

  mqttClient.on("message", (topic, payload) => {
    state.counters.total++;
    bumpTopic(state.counters, topic);

    const raw = payload.toString();
    const allowed = isAllowedIncomingTopic(topic);

    if (allowed) state.counters.allowed++;
    else state.counters.dropped++;

    console.log(`${allowed ? "ALLOW" : "DROP "} MQTT -> ${topic} ${raw}`);

    if (!allowed) return;

    const msg = { topic, raw, ts: Date.now() };
    try {
      msg.json = JSON.parse(raw);
    } catch {
      msg.json = null;
    }

    state.lastMessage = msg;

    // Route messages (same behavior as current code)
    if (topic === "robot/telemetry/fast") {
      state.lastFast = msg;
      io.emit("telemetry:fast", msg);
    } else if (topic === "robot/telemetry/slow") {
      state.lastSlow = msg;
      io.emit("telemetry:slow", msg);
    } else if (topic === TOPIC_STATE_DRIVE) {
      state.lastDriveState = msg;
      io.emit("drive:state", msg);
    } else if (topic === TOPIC_CMD_DRIVE_ACK) {
      state.lastDriveAck = msg;
      io.emit("drive:ack", msg);
    }
  });
}
