# Backend – Robot Control Server

Node.js backend that acts as a bridge between **MQTT** (robot / ESP32) and the **web frontend**.

It receives robot telemetry via MQTT, keeps an aggregated state, and exposes the data in **real time** to web clients using **Socket.IO** and HTTP endpoints.

The backend is also responsible for:
- sending control commands to the robot
- managing **tracking logic based on robot movement**

---

## Features

- Connects to an MQTT broker (e.g. Mosquitto)
- Subscribes to robot telemetry and state topics
- Keeps aggregated robot state
- Exposes data via:
  - HTTP REST endpoints
  - Socket.IO (real-time)
- Sends drive commands from frontend to robot
- Implements **tracking control logic**
- Publishes tracking state via MQTT
- Configuration via environment variables

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

```text
Frontend (React)
   ↓ HTTP / Socket.IO
Backend (Node.js)
   ↓ MQTT
Broker
   ↓ MQTT
ESP32 Robot
```

The backend is the **central coordination layer**:
- Frontend does not talk directly to MQTT
- ESP32 does not talk HTTP/WebSocket
- Backend manages logic and state

---

## Backend Structure

```
backend/
├─ index.js        # Application bootstrap
└─ src/
   ├─ http.js      # REST API
   ├─ ws.js        # Socket.IO
   └─ mqtt.js      # MQTT logic + tracking state machine
```

---

## Core Responsibilities

### `mqtt.js`
- MQTT connection and subscriptions
- Topic filtering
- Routing messages to frontend
- Maintaining internal state
- **Tracking state machine (movement + timer)**

### `http.js`
- `/health`
- `/telemetry/state`
- `/tracking` (NEW)

### `ws.js`
- Real-time telemetry updates
- Drive command forwarding

---

## Tracking Logic (IMPORTANT)

The backend implements a **state machine** that controls whether tracking is allowed.

### Inputs
- `robot/state/drive` (from ESP32)
- `robot/tracking/command` (from frontend)

### Internal state

```js
tracking = {
  robot_is_moving,
  stop_timestamp,
  tracking_allowed,
  tracking_active
}
```

### Rules

- If robot is moving → tracking OFF
- If robot stops → start timer
- If stopped for ≥ 5 seconds → tracking ON
- If tracking is manually disabled → tracking OFF

### Result

The backend publishes:

```
robot/tracking/status
```

---

## MQTT Topics

### Subscribed

- `robot/telemetry/*`
- `robot/state/drive`
- `robot/cmd/drive/ack`
- `robot/tracking/command` ✅ NEW

---

### Published

#### Drive commands
```
robot/cmd/drive
```

#### Tracking status (NEW)
```
robot/tracking/status
```

### Example

```json
{
  "tracking_allowed": true,
  "tracking_active": true,
  "robot_is_moving": false
}
```

---

## HTTP Endpoints

### Health

```
GET /health
```

---

### Telemetry State

```
GET /telemetry/state
```

Returns:

- fast telemetry
- slow telemetry
- drive state
- drive ACK
- **tracking state (NEW)**
- counters

---

### Tracking Control (NEW)

```
POST /tracking
```

### Payload

```json
{
  "tracking_allowed": true
}
```

### Behavior

- Publishes to MQTT:
  ```
  robot/tracking/command
  ```
- Updates tracking state in backend

---

## Real-time (Socket.IO)

### Events emitted

| Event | Description |
|------|------------|
| `telemetry:fast` | Fast telemetry |
| `telemetry:slow` | Slow telemetry |
| `drive:state` | Robot movement |
| `drive:ack` | ACK from robot |
| `tracking:status` | Tracking state (NEW) |

---

### Events received

| Event | Payload | Description |
|------|--------|------------|
| `cmd:drive` | `{ left, right }` | Drive command |

---

## MQTT Filtering

Allowed:
- telemetry
- drive state
- ACK
- tracking command

Dropped:
- other command topics (to avoid loops)

---

## Configuration

Create `.env`:

```bash
cp .env.example .env
```

### Variables

| Variable | Example |
|--------|--------|
| `PORT` | 3001 |
| `MQTT_URL` | mqtt://127.0.0.1:1883 |
| `MQTT_TOPIC` | robot/# |

---

## Run

```bash
npm install
npm run dev
```

---

## Testing

### View tracking status

```bash
mosquitto_sub -h localhost -t "robot/tracking/status" -v
```

---

### Enable tracking

```bash
curl -X POST http://localhost:3001/tracking \
  -H "Content-Type: application/json" \
  -d '{"tracking_allowed":true}'
```

---

### Disable tracking

```bash
curl -X POST http://localhost:3001/tracking \
  -H "Content-Type: application/json" \
  -d '{"tracking_allowed":false}'
```

---

## Design Principles

- Single source of truth (backend state)
- Separation of concerns
- Event-driven architecture
- MQTT for device communication
- HTTP/WebSocket for UI
- Deterministic tracking logic

---

## Current Status

- ✅ MQTT integration working
- ✅ Telemetry aggregation
- ✅ Drive control (frontend → robot)
- ✅ ACK system
- ✅ Tracking state machine implemented
- ✅ 5-second stop detection
- ✅ Tracking enable/disable via API
- 🔧 Vision + tracking integration (next step)

---

## Next Steps

- Integrate tracking with camera (Python)
- Add bounding box streaming
- Laser auto-targeting
- Video stream to frontend
- Advanced behaviors (avoidance, patrol)

---