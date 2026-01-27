# IoT Robot Platform

This repository contains a complete IoT-based robotic platform composed of multiple independent but interconnected components.

The project is designed to control and monitor a mobile robot using MQTT for communication and a web-based interface for real-time visualization and control.  
A Python-based simulator is included to allow development and testing without the physical robot.

---

## Project Structure

```
iot_robot/
├─ esp32/        # ESP32 firmware (PlatformIO)
├─ simulator/    # Python robot simulator (MQTT)
├─ backend/      # Node.js backend (MQTT ↔ Socket.IO)
├─ frontend/     # React frontend (Vite)
└─ README.md     # Project overview (this file)
```

---

## Components Overview

### 1. ESP32 Firmware (`esp32/`)
- Low-level robot control
- Motors, sensors, and actuators
- MQTT publisher/subscriber
- Developed with PlatformIO

📄 Documentation: `esp32/README.md`

---

### 2. Python Simulator (`simulator/`)
- Simulates robot telemetry and behavior
- Publishes sensor data via MQTT
- Subscribes to control commands
- Enables development without physical hardware

📄 Documentation: `simulator/README.md`

---

### 3. Backend Server (`backend/`)
- Node.js (Express) application
- Connects to MQTT broker (Mosquitto)
- Bridges MQTT ↔ Web using Socket.IO
- Handles telemetry distribution and control commands

📄 Documentation: `backend/README.md`

---

### 4. Frontend Web Application (`frontend/`)
- React application built with Vite
- Real-time telemetry visualization
- Robot control interface
- Communicates with backend via Socket.IO

📄 Documentation: `frontend/README.md`

---

## Architecture Overview

```
ESP32 / Simulator
        |
      MQTT
        |
   Backend (Node.js)
        |
    Socket.IO
        |
     Frontend (React)
```

---

## Technologies Used

- ESP32 / PlatformIO
- MQTT (Mosquitto)
- Node.js + Express
- Socket.IO
- React (Vite)
- Python

---

## Getting Started

Each component of the system is self-contained and provides its own setup and execution instructions:

- ESP32 firmware → `esp32/README.md`
- Python simulator → `simulator/README.md`
- Backend server → `backend/README.md`
- Frontend application → `frontend/README.md`

Follow the READMEs in this order for a first-time setup:
1. MQTT broker (Mosquitto)
2. Backend
3. Simulator or ESP32 firmware
4. Frontend

---

## Notes

- Sensitive configuration files (`.env`, `secrets.ini`) are **not committed**
- Example configuration files (`.env.example`, `secrets.example.ini`) are provided
- The system is designed to run locally on a PC and later be deployed on a Raspberry Pi

---

## License

This project is intended for educational and experimental purposes.
