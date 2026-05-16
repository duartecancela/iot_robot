# Backend – Robot Control Server

Node.js backend that acts as a bridge between **MQTT** (robot / ESP32) and the **web frontend**.

It receives robot telemetry via MQTT, keeps an aggregated state, and exposes the data in **real time** to web clients using **Socket.IO** and HTTP endpoints.

The backend is also responsible for:
- sending control commands to the robot
- managing **tracking control via frontend commands**

---

## Features

- Connects to an MQTT broker (e.g. Mosquitto)
- Subscribes to robot telemetry and state topics
- Keeps aggregated robot state
- Exposes data via:
  - HTTP REST endpoints
  - Socket.IO (real-time)
- Sends drive commands from frontend to robot
- Implements **manual tracking control**
- Publishes tracking state via MQTT
- MQTT logging to MongoDB
- Configuration via environment variables

---

## Tech Stack

- Node.js
- Express
- MQTT (`mqtt`)
- Socket.IO
- MongoDB (`mongodb`)
- dotenv

---

## MongoDB

The backend supports MongoDB integration for:

- MQTT logging
- telemetry history
- future analytics

Default local connection:

```text
mongodb://127.0.0.1:27017
```

Default database:

```text
iot_robot
```

### Collections

| Collection | Purpose |
|------------|----------|
| `mqtt_logs` | MQTT telemetry and event logging |

MongoDB setup instructions are available in:

```text
docs/mongodb_setup.md
```

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

```text
backend/
├─ index.js        # Application bootstrap
└─ src/
   ├─ db.js        # MongoDB connection
   ├─ http.js      # REST API
   ├─ ws.js        # Socket.IO
   └─ mqtt.js      # MQTT logic + tracking control
```

---

## Core Responsibilities

### `db.js`
- MongoDB connection management
- Database access layer

### `mqtt.js`
- MQTT connection and subscriptions
- Topic filtering
- Routing messages to frontend
- Maintaining internal state
- MQTT logging to MongoDB
- **Tracking control logic**

### `http.js`
- `/health`
- `/telemetry/state`
- `/tracking`
- `/logging`
- `/logging/status`
- `/test-log`

### `ws.js`
- Real-time telemetry updates
- Drive command forwarding

---

## Tracking Logic (IMPORTANT)

The backend implements a **tracking control mechanism** that determines whether object tracking should be active.

### Inputs
- `robot/tracking/command` (from frontend)

### Internal state

```js
tracking = {
  tracking_allowed,
  tracking_active
}
```

### Behavior

- Tracking is controlled **explicitly by the user** via the frontend
- When the user enables tracking:
  - backend updates internal state
  - publishes new tracking status via MQTT
- When the user disables tracking:
  - tracking is immediately stopped

### Result

The backend publishes:

```
robot/tracking/status
```

### Example

```json
{
  "tracking_allowed": true,
  "tracking_active": true
}
```

This approach provides **full manual control**, improving predictability and simplifying system behavior during testing and operation.

---

## MQTT Topics

### Subscribed

- `robot/telemetry/*`
- `robot/state/drive`
- `robot/cmd/drive/ack`
- `robot/tracking/command`

---

### Published

#### Drive commands
```
robot/cmd/drive
```

#### Tracking status
```
robot/tracking/status
```

### Example

```json
{
  "tracking_allowed": true,
  "tracking_active": true
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
- tracking state
- counters

---

### Tracking Control

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

- Controlled manually from frontend
- Publishes to MQTT:
  ```
  robot/tracking/command
  ```
- Updates backend tracking state

---

### Logging Status

```
GET /logging/status
```

Returns current logging state.

### Example

```json
{
  "ok": true,
  "logging": {
    "enabled": true,
    "startedAt": 1778944524741
  }
}
```

---

### Logging Control

```
POST /logging
```

### Payload

```json
{
  "enabled": true
}
```

### Behavior

- Enables or disables MQTT logging to MongoDB
- Updates backend logging state

---

### Test MongoDB Insert

```
POST /test-log
```

Used to validate MongoDB connectivity and document insertion.

---

## Real-time (Socket.IO)

### Events emitted

| Event | Description |
|------|------------|
| `telemetry:fast` | Fast telemetry |
| `telemetry:slow` | Slow telemetry |
| `drive:state` | Robot movement |
| `drive:ack` | ACK from robot |
| `tracking:status` | Tracking state |

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

## Install Dependencies

```bash
npm install
```

---

## Run

```bash
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

### Enable logging

```bash
curl -X POST http://localhost:3001/logging \
  -H "Content-Type: application/json" \
  -d '{"enabled":true}'
```

---

### Disable logging

```bash
curl -X POST http://localhost:3001/logging \
  -H "Content-Type: application/json" \
  -d '{"enabled":false}'
```

---

### Check logging status

```bash
curl http://localhost:3001/logging/status
```

---

### Publish MQTT test message

```bash
mosquitto_pub -h localhost \
  -t robot/telemetry/fast \
  -m '{"test":true,"value":123}'
```

---

## Design Principles

- Single source of truth (backend state)
- Separation of concerns
- Event-driven architecture
- MQTT for device communication
- HTTP/WebSocket for UI
- Explicit user control over tracking behavior

---

## Current Status

- ✅ MQTT integration working
- ✅ Telemetry aggregation
- ✅ Drive control (frontend → robot)
- ✅ ACK system
- ✅ Manual tracking control via frontend
- ✅ Tracking enable/disable via API
- ✅ MongoDB integration
- ✅ MQTT logging to MongoDB
- ✅ Logging enable/disable API
- 🔧 Vision + tracking integration (next step)

---

## Next Steps

- Integrate logging controls into frontend
- Add log visualization dashboard
- Integrate tracking with camera (Python)
- Add bounding box streaming
- Laser auto-targeting
- Video stream to frontend
- Advanced behaviors (avoidance, patrol)