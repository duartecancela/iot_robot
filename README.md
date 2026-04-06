# IoT Robot Platform

A modular MQTT-based robotic platform composed of firmware, edge runtime,
backend, frontend and simulation layers.

The system enables real-time monitoring and control of a mobile robot
using MQTT as the communication backbone and a web-based interface
for visualization and command execution.

---

## Project Structure

```text
iot_robot/
├─ .vscode/       # VSCode settings
├─ backend/       # Node.js backend (MQTT ↔ Socket.IO bridge)
├─ docs/          # Documentation (diagrams, notes, reports)
├─ esp32/         # ESP32 firmware (PlatformIO)
├─ frontend/      # React frontend (Vite)
├─ rpi/           # Raspberry Pi runtime (camera, servos, vision)
├─ shared/        # Shared configuration
│  └─ config/
│     └─ defaults.json
├─ simulator/     # Python robot simulator (MQTT)
├─ start_dev.sh   # Dev script (start backend + frontend)
├─ stop_dev.sh    # Dev script (stop services)
├─ .gitignore
└─ README.md
```

---

## System Architecture

The system follows a layered, decoupled architecture:

```text
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

```text
iotrobot.local
```

This avoids hardcoded IP addresses and allows the system to operate across
different networks (home Wi-Fi, hotspot, lab networks, etc.).

Example access points:

Frontend:
```text
http://iotrobot.local:5173
```

Backend API:
```text
http://iotrobot.local:3001
```

MQTT Broker:
```text
mqtt://iotrobot.local:1883
```

Raspberry Pi video stream:
```text
http://iotrobot.local:8080/stream.mjpg
```

Raspberry Pi detections endpoint:
```text
http://iotrobot.local:8080/detections
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
- MQTT-controlled tracking state
- MJPEG stream + JSON detections endpoint

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
- Publishes tracking state for Raspberry Pi

See: `backend/README.md`

---

### Frontend (`frontend/`)
React (Vite) web interface:
- Real-time telemetry visualization
- Robot control interface
- Tracking enable/disable button
- Communicates with backend via Socket.IO + HTTP

See: `frontend/README.md`

---

## Shared Configuration

Default configuration is defined in:

```text
shared/config/defaults.json
```

Configuration resolution order:

```text
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
```text
http://iotrobot.local:5173
```

Backend:
```text
http://iotrobot.local:3001
```

These scripts are intended only for development.

Production deployment will later use dedicated system services
on the Raspberry Pi.

---

## Manual Startup

This section documents the current manual commands required to start
each layer during development.

### 1) Start MQTT broker

On the Raspberry Pi or machine running Mosquitto:

```bash
sudo systemctl start mosquitto
sudo systemctl status mosquitto
```

Optional test:

```bash
mosquitto_sub -h localhost -t "robot/#" -v
```

---

### 2) Start backend

From the project root:

```bash
cd backend
npm install
npm run dev
```

Expected endpoint:

```text
http://iotrobot.local:3001
```

---

### 3) Start frontend

From the project root:

```bash
cd frontend
npm install
npm run dev -- --host
```

Expected endpoint:

```text
http://iotrobot.local:5173
```

---

### 4) Start Raspberry Pi camera streaming

From the project root:

```bash
cd rpi
source venv/bin/activate
python -m camera.camera_stream
```

Or with explicit target selection:

```bash
cd rpi
source venv/bin/activate
TARGET=cell_phone python -m camera.camera_stream
```

Alternative examples:

```bash
TARGET=bird python -m camera.camera_stream
TARGET=all python -m camera.camera_stream
```

Available endpoints:

Stream:
```text
http://iotrobot.local:8080/stream.mjpg
```

Detections:
```text
http://iotrobot.local:8080/detections
```

Health:
```text
http://iotrobot.local:8080/health
```

---

### 5) Start Raspberry Pi tracking controller

In another terminal:

```bash
cd rpi
source venv/bin/activate
python -m tracking.tracking_controller
```

This process:

- subscribes to `robot/tracking/status`
- waits for tracking state updates from backend
- only runs servo tracking when `tracking_active == true`
- recenters servos when tracking state changes
- keeps the camera stream running independently

---

### 6) Optional: start simulator instead of ESP32

From the project root:

```bash
cd simulator
python3 simulator.py
```

Use this when testing backend/frontend without the physical robot.

---

## Tracking Control Flow

Tracking is now controlled manually from the frontend button.

Flow:

```text
Frontend button
   ↓
POST /tracking
   ↓
MQTT -> robot/tracking/command
   ↓
Backend recomputes tracking state
   ↓
MQTT -> robot/tracking/status
   ↓
Raspberry Pi tracking controller enables/disables tracking
```

Current behavior:

- Video streaming remains always available
- Tracking only runs when enabled from frontend
- When tracking is disabled, servos return to the initial position
- Tracking state is published with MQTT retain enabled

---

## Raspberry Pi Wiring Notes

## PCA9685

- VCC → 3.3V (RPi)
- GND → GND (Pin 6, shared)
- SDA → GPIO2 (Pin 3)
- SCL → GPIO3 (Pin 5)

## Servos

- External 5V supply required
- Do not power servos directly from Raspberry Pi
- Common GND mandatory

## Laser (MOSFET)

- GPIO18 (Pin 12) → Gate
- GND → Common GND
- Laser powered externally (5V)

---

## Core Technologies

- ESP32 / PlatformIO
- Raspberry Pi (Python / OpenCV)
- MQTT (Mosquitto)
- Node.js (Express)
- Socket.IO
- React (Vite)
- Python
- Picamera2
- PCA9685

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
- A future startup script can automate backend + frontend + Raspberry Pi services

---

## License

Educational and experimental purposes.