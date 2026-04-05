# IoT Robot Platform

A modular MQTT-based robotic platform composed of firmware, edge runtime,
backend, frontend and simulation layers.

The system enables real-time monitoring and control of a mobile robot
using MQTT as the communication backbone and a web-based interface
for visualization and command execution.

---

## Project Structure

```
iot_robot/
├─ esp32/         # ESP32 firmware (PlatformIO)
├─ rpi/           # Raspberry Pi runtime (camera, servos, vision)
├─ simulator/     # Python robot simulator (MQTT)
├─ backend/       # Node.js backend (MQTT ↔ Socket.IO bridge)
├─ frontend/      # React frontend (Vite)
├─ shared/        # Shared configuration
│  └─ config/
│     └─ defaults.json
└─ README.md
```

---

## System Architecture

The system follows a layered, decoupled architecture:

```
ESP32 (motors/sensors)     Raspberry Pi (vision/servos)
            |                       |
            +---------- MQTT -------+
                        |
                Mosquitto Broker
                        |
                   Backend (Node.js)
                        |
                    Socket.IO
                        |
                    Frontend (React)
```

Each layer is independent and replaceable, allowing development and testing
without requiring physical hardware.

---

## Network Access

When running the system on a Raspberry Pi, services are accessed using the
hostname:

```
iotrobot.local
```

This avoids hardcoded IP addresses and allows the system to operate across
different networks (home Wi-Fi, hotspot, lab networks, etc.).

Example access points:

Frontend:
```
http://iotrobot.local:5173
```

Backend API:
```
http://iotrobot.local:3001
```

MQTT Broker:
```
mqtt://iotrobot.local:1883
```

All components (ESP32 firmware, simulator, backend and frontend)
use this hostname instead of fixed IP addresses.

---

## Components

### ESP32 Firmware (`esp32/`)
Low-level robot control:
- Motor drivers
- Sensor acquisition
- MQTT publish/subscribe
- PlatformIO-based firmware

See: `esp32/README.md`

---

### Raspberry Pi Runtime (`rpi/`)
Edge computation layer:
- Camera capture
- Object detection
- Servo pan/tilt control
- Laser / actuator control
- Optional MQTT integration

This layer performs high-level processing that is not suitable
for microcontroller execution.

See: `rpi/README.md`

---

### Python Simulator (`simulator/`)
MQTT-based robot emulator:
- Publishes telemetry
- Receives drive commands
- Mirrors ESP32 MQTT behavior
- Used for development and testing

See: `simulator/README.md`

---

### Backend (`backend/`)
Node.js (Express) application:
- Connects to MQTT broker
- Bridges MQTT ↔ Web (Socket.IO)
- Aggregates telemetry state
- Exposes HTTP + real-time endpoints

See: `backend/README.md`

---

### Frontend (`frontend/`)
React (Vite) web interface:
- Real-time telemetry visualization
- Robot control interface
- Communicates with backend via Socket.IO

See: `frontend/README.md`

---

## Shared Configuration

Default configuration is defined in:

```
shared/config/defaults.json
```

Configuration resolution order:

```
Environment variables → shared defaults → internal fallback
```

This ensures portability across:

- PC development
- Raspberry Pi deployment
- Simulator usage
- Physical hardware execution

---

## Development Startup

For convenience during development, helper scripts can start both the
backend and frontend development servers.

Start development environment:

```bash
./start_dev.sh
```

Stop development environment:

```bash
./stop_dev.sh
```

Default development endpoints:

Frontend:
```
http://iotrobot.local:5173
```

Backend:
```
http://iotrobot.local:3001
```

These scripts are intended only for development.

Production deployment will later use dedicated system services
on the Raspberry Pi.

---

## Core Technologies

- ESP32 / PlatformIO
- Raspberry Pi (Python / OpenCV)
- MQTT (Mosquitto)
- Node.js (Express)
- Socket.IO
- React (Vite)
- Python

---

## Deployment Targets

- **Development:** PC + Simulator
- **Edge Runtime:** Raspberry Pi
- **Production Robot:** ESP32 + Raspberry Pi

No structural changes are required when switching environments.

---

## Notes

- Sensitive files (`.env`, `secrets.ini`) are not committed
- Example configuration files are provided
- Designed for modular expansion and future hardware integration

---

## License

Educational and experimental purposes.