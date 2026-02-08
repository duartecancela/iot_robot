# Backend – Robot Control Server

Node.js backend that acts as a bridge between **MQTT** (robot / simulator) and the **web frontend**.

It receives robot telemetry via MQTT, keeps an aggregated state, and exposes the data in **real time** to web clients using **Socket.IO** and simple HTTP endpoints.

The backend is also responsible for **sending control commands** from the web UI to the robot via MQTT.

---

## Features

- Connects to an MQTT broker (e.g. Mosquitto)
- Subscribes to robot telemetry topics
- Keeps the last received messages and aggregated state
- Routes telemetry and state to:
  - HTTP REST endpoints
  - Socket.IO (real-time)
- Receives drive commands from frontend and publishes them to MQTT
- Configuration via environment variables with shared defaults
- Clear separation of concerns (HTTP / WebSocket / MQTT)

---

## Tech Stack

- Node.js
- Express
- MQTT (`mqtt`)
- Socket.IO
- dotenv

---

## Requirements

- Node.js >= 18
- MQTT broker (e.g. Mosquitto)

---

## Architecture Overview

```
Frontend (React)
   ↓ Socket.IO
Backend (Node.js)
   ↓ MQTT
Broker
   ↓ MQTT
Robot (ESP32) / Simulator
```

The backend is the **single integration point**:
- Frontend never talks directly to MQTT
- Robot never talks directly to WebSocket or HTTP

---

## Backend Structure

The backend is split into **three logical modules**:

```
backend/
├─ index.js        # Application bootstrap (wires everything)
└─ src/
   ├─ http.js      # Express HTTP server and REST endpoints
   ├─ ws.js        # Socket.IO real-time communication
   └─ mqtt.js      # MQTT connection, subscriptions and routing
```

### Responsibilities

- `http.js`
  - `/health`
  - `/telemetry/state`
  - JSON responses for debugging and monitoring

- `ws.js`
  - Handles WebSocket connections
  - Sends last known telemetry on connect
  - Receives `cmd:drive` from frontend
  - Emits real-time events to clients

- `mqtt.js`
  - Connects and subscribes to MQTT broker
  - Filters allowed topics
  - Routes messages to Socket.IO
  - Updates shared backend state

---

## Configuration strategy (important)

The backend uses a **two-level configuration approach**:

1. **Environment variables (`.env`)** – local overrides (machine-specific)
2. **Shared defaults** – `shared/config/defaults.json`

Resolution order:
```
.env  →  shared/config/defaults.json  →  internal fallback
```

This makes the project portable across different machines  
(PC, Raspberry Pi, school lab).

---

## Environment variables

Create a `.env` file based on the example:

```bash
cp .env.example .env
```

### Supported variables

| Variable | Description | Example |
|--------|------------|--------|
| `PORT` | HTTP server port | `3001` |
| `MQTT_URL` | Full MQTT URL (optional) | `mqtt://127.0.0.1:1883` |
| `MQTT_HOST` | MQTT broker host (used if `MQTT_URL` not set) | `127.0.0.1` |
| `MQTT_PORT` | MQTT broker port (used if `MQTT_URL` not set) | `1883` |
| `MQTT_TOPIC` | MQTT topic wildcard to subscribe | `robot/#` |
| `MQTT_TOPIC_STATE_DRIVE` | Robot drive state topic | `robot/state/drive` |
| `MQTT_TOPIC_CMD_DRIVE` | Drive command topic | `robot/cmd/drive` |
| `MQTT_TOPIC_CMD_DRIVE_ACK` | Drive command acknowledgement | `robot/cmd/drive/ack` |

If a variable is not defined, the backend falls back to  
`shared/config/defaults.json`.

---

## Setup

```bash
npm install
cp .env.example .env
```

Edit `.env` only if you need to override defaults.

---

## Run

```bash
npm run dev
```

or:

```bash
node index.js
```

Backend will start at:

```
http://localhost:3001
```

---

## HTTP Endpoints

### Health check

```
GET /health
```

Returns basic status and MQTT configuration info.

---

### Aggregated telemetry state

```
GET /telemetry/state
```

Returns:
- last fast telemetry message
- last slow telemetry message
- last drive state
- last drive acknowledgement
- message counters
- timestamp

This endpoint is useful for:
- debugging
- initial UI state
- monitoring backend activity

---

## Real-time (Socket.IO)

### Events emitted by backend

| Event | Description |
|------|------------|
| `telemetry:fast` | High-frequency telemetry |
| `telemetry:slow` | Low-frequency telemetry |
| `drive:state` | Current robot drive state |
| `drive:ack` | Acknowledgement from robot |
| `mqtt:message` | Raw MQTT message (debug) |

On connection, the backend immediately sends the **last known values** for all channels.

---

### Events received by backend

| Event | Payload | Description |
|------|--------|------------|
| `cmd:drive` | `{ left, right }` | Drive command from frontend |

- Values are clamped to `-255 .. 255`
- Backend publishes the command to:
  ```
  robot/cmd/drive
  ```
- Backend emits confirmation back to sender:
  ```
  cmd:drive:sent
  ```

---

## MQTT Topic Handling

### Subscribed topics

- `robot/telemetry/*`
- `robot/state/drive`
- `robot/cmd/drive/ack`

### Dropped topics

- `robot/cmd/*` (except acknowledgements)

This prevents feedback loops and accidental command echoing.

---

## Notes for academic context

- Clear **separation of concerns**
- Backend acts as a **protocol gateway**
- Same backend works with:
  - real ESP32 robot
  - Python simulator
- MQTT filtering avoids unsafe command loops
- Suitable for:
  - IoT
  - Cyber-Physical Systems
  - Web of Things
  - Distributed Systems projects

---

## Next components

- Python robot simulator (MQTT publisher)
- React + Vite frontend (Socket.IO consumer)
- ESP32 firmware (real robot)
