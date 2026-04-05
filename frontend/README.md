# Frontend – IoT Robot Web Interface

Web-based frontend for the IoT Robot Platform.

This application provides a real-time interface to visualize robot telemetry,
control movement, and manage **tracking behavior**.

It communicates with the backend using:
- **HTTP (REST)** → aggregated state + commands
- **Socket.IO** → real-time updates

---

## Tech Stack

- React
- Vite
- Tailwind CSS
- Socket.IO client
- Heroicons (`@heroicons/react`)

---

## Features

- Real-time telemetry dashboard
- Differential drive joystick control
- Drive ACK visualization
- **Tracking state visualization (NEW)**
- **Tracking enable/disable control (NEW)**
- Responsive UI with Tailwind CSS
- Clean and modular component structure

---

## Requirements

- Node.js >= 18
- Backend server running
- MQTT broker (used indirectly via backend)

---

## Configuration

Create a `.env` file:

```bash
cp .env.example .env
```

### Variables

| Variable | Description | Example |
|--------|------------|--------|
| `VITE_BACKEND_URL` | Backend base URL | `http://localhost:3001` |

> ⚠️ Must start with `VITE_`

---

## Setup

```bash
npm install
```

---

## Run

```bash
npm run dev
```

Default:

```
http://localhost:5173
```

---

## Project Structure

```
frontend/
├─ src/
│  ├─ components/
│  │   ├─ TelemetryState.jsx   # Telemetry + tracking UI
│  │   ├─ DriveJoystick.jsx    # Robot control
│  ├─ App.jsx
│  ├─ main.jsx
│  └─ index.css
├─ public/
├─ .env.example
├─ vite.config.js
└─ README.md
```

---

## Architecture Overview

```
Frontend (React)
   ↓ HTTP / Socket.IO
Backend (Node.js)
   ↓ MQTT
Broker
   ↓ MQTT
ESP32 Robot
```

- Frontend does NOT talk MQTT directly
- Backend is the only integration layer

---

## Telemetry Dashboard

The `TelemetryState` component displays:

### Sensors
- ToF (front-left / front-right)
- IMU (pitch, roll)
- BME280 (temperature, humidity, pressure)

### Drive
- Applied motor values
- ACK status

### Tracking (NEW)

| Field | Description |
|------|------------|
| `Moving` | Robot is currently moving |
| `Tracking Active` | Tracking system is running |
| `Tracking Allowed` | Manual enable/disable state |

---

## Tracking Control (NEW)

The frontend allows enabling/disabling tracking via:

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

- Sends command to backend
- Backend publishes:
  ```
  robot/tracking/command
  ```
- Backend updates tracking state
- UI reflects new state automatically

---

## Tracking Logic (System Overview)

Tracking is controlled by the backend based on:

- Robot movement (`robot/state/drive`)
- Manual control (`tracking_allowed`)
- Stop timer (5 seconds)

### Rules

- Moving → tracking OFF
- Stopped < 5s → tracking OFF
- Stopped ≥ 5s → tracking ON (if allowed)

The frontend **only displays and controls the state**,  
it does not implement the logic.

---

## Drive Joystick

### Behavior

- `x` → turning
- `y` → forward/backward

```
left  = y + x
right = y - x
```

- Range: `-255 .. 255`
- Sends updates ~20 Hz
- Auto stop on release

---

## Backend Communication

### HTTP (Polling)

```
GET /telemetry/state
```

Used for:
- dashboard data
- tracking state
- initial load

---

### HTTP (Command)

```
POST /tracking
```

Used for:
- enabling/disabling tracking

---

### Socket.IO (Real-time)

#### Incoming events

| Event | Description |
|------|------------|
| `telemetry:fast` | Fast telemetry |
| `telemetry:slow` | Slow telemetry |
| `drive:state` | Drive state |
| `drive:ack` | ACK |
| `tracking:status` | Tracking updates (NEW) |

#### Outgoing events

| Event | Payload |
|------|--------|
| `cmd:drive` | `{ left, right }` |

---

## Network Access

To allow access from other devices:

```js
server: {
  host: true,
  allowedHosts: 'all'
}
```

Access via:

```
http://iotrobot.local:5173
```

---

## Styling

- Tailwind CSS
- Utility-first
- Compact telemetry UI
- Semantic colors:
  - amber → ToF
  - slate → IMU
  - emerald → environment
  - rose → drive
  - indigo → tracking

---

## Development Notes

- Frontend is **stateless regarding safety**
- All validation happens in backend
- Can be used with:
  - real robot
  - simulator
  - MQTT CLI testing

---

## Troubleshooting

- Blank page:
  ```bash
  npm install
  ```

- Backend not reachable:
  - Check `VITE_BACKEND_URL`

- No telemetry:
  - Confirm backend + MQTT running

---

## Current Status

- ✅ Telemetry dashboard
- ✅ Drive control (joystick)
- ✅ ACK visualization
- ✅ Tracking state display
- ✅ Tracking enable/disable button
- 🔧 Camera + vision integration (next step)

---

## Next Steps

- Camera stream in frontend
- Object detection overlay
- Laser targeting UI
- Mobile control improvements
- Charts / history

---

## License

Educational / experimental use