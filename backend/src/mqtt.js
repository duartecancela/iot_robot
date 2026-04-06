// src/mqtt.js

export function isAllowedIncomingTopicFactory({
  TOPIC_STATE_DRIVE,
  TOPIC_CMD_DRIVE_ACK,
}) {
  return function isAllowedIncomingTopic(topic) {
    if (!topic) return false;
    if (topic.startsWith("robot/telemetry/")) return true;
    if (topic === TOPIC_STATE_DRIVE) return true;
    if (topic === TOPIC_CMD_DRIVE_ACK) return true;
    if (topic === "robot/tracking/command") return true;
    return false;
  };
}

export function bumpTopic(counters, topic) {
  counters.byTopic[topic] = (counters.byTopic[topic] || 0) + 1;
}

function ensureTrackingState(state) {
  if (!state.tracking) {
    state.tracking = {
      robot_is_moving: false,
      stop_timestamp: null,
      tracking_allowed: false,
      tracking_active: false,
    };
  }
}

function buildTrackingStatus(state) {
  return {
    tracking_allowed: state.tracking.tracking_allowed,
    tracking_active: state.tracking.tracking_active,
    robot_is_moving: state.tracking.robot_is_moving,
  };
}

function publishTrackingStatus(mqttClient, state, io) {
  const payload = buildTrackingStatus(state);

  mqttClient.publish(
    "robot/tracking/status",
    JSON.stringify(payload),
    { retain: true }
  );

  io.emit("tracking:status", {
    topic: "robot/tracking/status",
    json: payload,
    ts: Date.now(),
  });
}

function recomputeTrackingState(state) {
  ensureTrackingState(state);

  const prevActive = state.tracking.tracking_active;

  // Tracking is controlled only by the frontend button.
  state.tracking.tracking_active = state.tracking.tracking_allowed;

  return prevActive !== state.tracking.tracking_active;
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

  ensureTrackingState(state);

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

    mqttClient.subscribe(
      [TOPIC_STATE_DRIVE, TOPIC_CMD_DRIVE_ACK, "robot/tracking/command"],
      (err) => {
        if (err) {
          console.log("MQTT subscribe (required) error:", err.message);
        } else {
          console.log(
            "MQTT subscribed (required):",
            TOPIC_STATE_DRIVE,
            TOPIC_CMD_DRIVE_ACK,
            "robot/tracking/command"
          );

          recomputeTrackingState(state);
          publishTrackingStatus(mqttClient, state, io);
        }
      }
    );
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

    if (topic === "robot/telemetry/fast") {
      state.lastFast = msg;
      io.emit("telemetry:fast", msg);
    } else if (topic === "robot/telemetry/slow") {
      state.lastSlow = msg;
      io.emit("telemetry:slow", msg);
    } else if (topic === TOPIC_STATE_DRIVE) {
      state.lastDriveState = msg;
      io.emit("drive:state", msg);

      const drive = msg.json || {};
      state.tracking.robot_is_moving =
        typeof drive.moving === "boolean"
          ? drive.moving
          : (drive.left !== 0 || drive.right !== 0);

      publishTrackingStatus(mqttClient, state, io);
    } else if (topic === "robot/tracking/command") {
      const cmd = msg.json || {};

      if (typeof cmd.tracking_allowed === "boolean") {
        state.tracking.tracking_allowed = cmd.tracking_allowed;
        recomputeTrackingState(state);
        publishTrackingStatus(mqttClient, state, io);
        console.log("Tracking allowed set to:", cmd.tracking_allowed);
      }
    } else if (topic === TOPIC_CMD_DRIVE_ACK) {
      state.lastDriveAck = msg;
      io.emit("drive:ack", msg);
    }
  });
}