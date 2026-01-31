# Backend – Robot Control Server

Node.js backend that acts as a bridge between **MQTT** (robot / simulator) and the **web frontend**.

It receives robot telemetry via MQTT, keeps an aggregated state, and exposes the data in **real time** to web clients using **Socket.IO** and simple HTTP endpoints.

---

## Features

- Connects to an MQTT broker (e.g. Mosquitto)
- Subscribes to robot telemetry topics
- Keeps the last received message and aggregated state
- Exposes telemetry to:
  - HTTP REST endpoints
  - Socket.IO (real-time)
- Configuration via environment variables with shared defaults

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

## Configuration strategy (important)

The backend uses a **two-level configuration approach**:

1. **Environment variables (`.env`)** – local overrides (machine-specific)
2. **Shared defaults** – `shared/config/defaults.json`

Resolution order:
```
.env  →  shared/config/defaults.json  →  internal fallback
```

This makes the project portable across different machines (PC, Raspberry Pi, school lab).

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
| `MQTT_TOPIC` | MQTT topic wildcard to subscribe | `robot/telemetry/#` |

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

Example response:
```json
{
  "ok": true,
  "mqttUrl": "mqtt://127.0.0.1:1883",
  "topic": "robot/telemetry/#"
}
```

---

### Last received message
```
GET /telemetry/last
```

Returns the most recent MQTT message received.

---

### Aggregated telemetry state
```
GET /telemetry/state
```

Returns:
- last fast telemetry
- last slow telemetry
- timestamp
- MQTT configuration info

---

## Real-time (Socket.IO)

- Event: `telemetry`
- Payload: last received MQTT message (parsed JSON + raw payload)

On connection, the backend immediately sends the last known message (if any).

---

## Notes for academic context

- The backend follows a **clean separation of concerns**
- Uses **shared configuration defaults** to avoid duplication
- Supports both **simulated** and **real robot** telemetry without code changes
- Designed to scale from local development to Raspberry Pi deployment

---

## Next components

- Python robot simulator (MQTT publisher)
- React + Vite frontend (Socket.IO consumer)
- ESP32 firmware (real robot)

