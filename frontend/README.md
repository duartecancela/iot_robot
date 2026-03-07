# Frontend – IoT Robot Web Interface

Web-based frontend for the IoT Robot Platform.

This application provides a real-time interface to visualize robot telemetry and send control commands.  
It communicates with the backend using **Socket.IO** and follows a clean, component-based architecture.

---

## Tech Stack

- React
- Vite
- Tailwind CSS
- **Socket.IO client** (real-time communication with backend)
- **Heroicons (`@heroicons/react`)** – UI icons

---

## Features

- Real-time telemetry visualization
- **Real-time drive control via joystick**
- Clean layout with reusable components
- Responsive UI using Tailwind CSS
- Environment-based configuration (Vite)
- Minimalist, compact dashboard with semantic icons

---

## Requirements

- Node.js >= 18
- Backend server running (see `backend/README.md`)
- MQTT broker running (used indirectly via backend)

---

## Configuration

The frontend uses **Vite environment variables**.

Create a `.env` file based on the example:

```bash
cp .env.example .env
```

### Environment variables

| Variable | Description | Example |
|--------|------------|--------|
| `VITE_BACKEND_URL` | Backend base URL | `http://localhost:3001` |

> ⚠️ All frontend environment variables **must start with `VITE_`**.

The `.env` file is not committed.  
Default values are documented in `.env.example`.

---

## Setup

Install dependencies:

```bash
npm install
```

This frontend relies on:
- `socket.io-client` for real-time bidirectional communication
- `@heroicons/react` for UI icons

Make sure all dependencies are installed before running the application.

---

## Run (Development)

Start the development server:

```bash
npm run dev
```

Vite will print the local URL, usually:

```
http://localhost:5173
```

---

## Project Structure

```
frontend/
├─ src/
│  ├─ components/     # Reusable UI components (Telemetry, Joystick, etc.)
│  ├─ App.jsx         # Main application layout
│  ├─ main.jsx        # Application entry point
│  └─ index.css       # Tailwind CSS entry
├─ public/
├─ .env.example
├─ vite.config.js
└─ README.md
```

---

## Network Access (Development)

When running the frontend on a Raspberry Pi or another machine and accessing it from
other devices on the same network (PC, phone, tablet), Vite must allow external hosts.

By default, Vite blocks unknown hosts for security reasons.

### Vite configuration

Edit `vite.config.js` and enable network access:

```javascript
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

export default defineConfig({
  plugins: [
    react(),
    tailwindcss(),
  ],
  server: {
    host: true,
    allowedHosts: 'all'
  }
})
```

Explanation:

| Option | Purpose |
|------|--------|
| `host: true` | Allows Vite to listen on all network interfaces (`0.0.0.0`) |
| `allowedHosts` | Allows external hosts such as `iotrobot.local` |

### Access from other devices

When running on the Raspberry Pi, the frontend can be accessed from any device on the same network:

```
http://iotrobot.local:5173
```

Example devices:

- Desktop PC
- Laptop
- Smartphone
- Tablet

This is useful for testing the robot interface on mobile devices.

## Styling

Styling is handled with **Tailwind CSS** using the official Vite plugin.

- Utility-first approach
- No custom CSS framework
- Compact, responsive layout
- Soft, low-contrast color palette for telemetry dashboards

Tailwind is enabled via:

```css
@import "tailwindcss";
```

---

## Backend Communication

### Architecture Overview

The frontend **never communicates directly with MQTT**.

All communication follows this flow:

```
Frontend (React)
   ↓ Socket.IO
Backend (Node.js)
   ↓ MQTT
Broker
   ↓ MQTT
Robot (ESP32)
```

This keeps credentials, topics, and validation logic centralized in the backend.

---

### Socket.IO Events

**Outbound (Frontend → Backend)**

| Event | Payload | Description |
|-----|--------|------------|
| `cmd:drive` | `{ left: number, right: number }` | Send motor speeds |

Motor values:
- Range: `-255 .. 255`
- Left and right motors are sent independently

**Inbound (Backend → Frontend)**

| Event | Payload | Description |
|------|--------|------------|
| `telemetry` | object | Robot telemetry snapshot |
| `cmd:drive:sent` | `{ ok, left, right, ts }` | Acknowledgement of drive command |
| `mqtt:message` | `{ topic, raw, ts }` | Raw MQTT messages (debug / advanced UI) |

---

## Drive Joystick

The drive joystick provides **real-time differential drive control**.

### Behaviour

- Joystick axes:
  - `x` → turning (left / right)
  - `y` → forward / backward
- Mapping:
  ```
  left  = y + x
  right = y - x
  ```
- Output values are:
  - clamped to `-255 .. 255`
  - sent at a controlled rate (~20 Hz)
- Releasing the joystick automatically sends `(0, 0)`

### Notes

- Rate limiting avoids flooding the backend and MQTT broker
- Commands are only sent when values change
- Backend performs final validation and clamping

---

## Development Notes

- The frontend does **not** require MQTT libraries
- Socket.IO is the only real-time dependency
- All robot control passes through backend validation
- The UI can be used with:
  - real robot
  - simulator
  - MQTT CLI testing (`mosquitto_pub`)
- Frontend remains stateless regarding robot safety

---

## Troubleshooting

- Blank page:
  ```bash
  npm install
  ```
- Connection issues:
  - Confirm `VITE_BACKEND_URL`
  - Ensure backend is running and reachable
- Icon-related errors:
  - Verify `@heroicons/react` version matches imports

---

## Next Steps

- Charts and historical telemetry
- Camera + pan/tilt control UI
- Laser / deterrence controls
- Connection status & latency indicators
- Mobile-friendly control layout

---

## License

This project is intended for educational and experimental purposes.
