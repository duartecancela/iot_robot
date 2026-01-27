# Backend – Robot Control Server

Node.js backend that:
- Connects to MQTT (Mosquitto)
- Receives robot telemetry
- Exposes real-time data to the frontend via Socket.IO
- Sends motor commands to the robot via MQTT

## Requirements
- Node.js >= 18
- MQTT broker (Mosquitto)

## Setup

```bash
npm install
cp .env.example .env
