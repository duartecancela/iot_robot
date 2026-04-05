# MQTT Topics - IoT Robot 4WD

## 📌 Overview

This document describes the MQTT topics used in the IoT Robot project.

It reflects the **current implemented system**, including:
- ESP32 telemetry and drive state
- Backend tracking logic
- Frontend control integration

---

## ✅ Implemented Topics

---

### 🔹 Control (Frontend → Backend → ESP32)

#### Drive Command
```
robot/cmd/drive
```

- Sent by backend (from frontend input)
- Controls robot movement

**Example:**
```json
{"left":120,"right":120}
```

---

#### Drive ACK
```
robot/cmd/drive/ack
```

- Sent by ESP32
- Confirms command reception and execution

**Example:**
```json
{"ok":true,"left":120,"right":120}
```

---

### 🔹 Robot State

#### Applied Drive State (ESP32 → Backend)

```
robot/state/drive
```

- **Single source of truth for robot movement**
- Published by ESP32 (`main.cpp`)
- Retained topic

**Example:**
```json
{
  "left": 100,
  "right": 100,
  "moving": true,
  "ts": 112210
}
```

### Fields

| Field | Description |
|------|------------|
| `left` | Applied left motor value |
| `right` | Applied right motor value |
| `moving` | `true` if robot is moving |
| `ts` | Timestamp (`millis()`) |

---

### 🔹 Telemetry

#### Fast telemetry
```
robot/telemetry/fast
```

**Content:**
- ToF sensors
- IMU

```json
{
  "tof": { "fl": 329, "fr": 381 },
  "imu": { "pitch": 1.31, "roll": 1.61 }
}
```

---

#### Slow telemetry
```
robot/telemetry/slow
```

**Content:**
- BME280 data

```json
{
  "bme": { "t": 21.35, "h": 52.28, "p": 992.64 }
}
```

---

## 🔹 Tracking System (IMPLEMENTED)

---

### Tracking Status (Backend → Frontend)

```
robot/tracking/status
```

Published by backend.

**Example:**
```json
{
  "tracking_allowed": true,
  "tracking_active": true,
  "robot_is_moving": false
}
```

---

### Tracking Command (Frontend → Backend)

```
robot/tracking/command
```

Used to enable/disable tracking.

**Example:**
```json
{
  "tracking_allowed": true
}
```

---

## 🧠 Tracking Logic (Backend)

Tracking is **NOT decided by the ESP32**.

It is implemented in the backend using:

- `robot/state/drive`
- `tracking_allowed`
- stop timer

---

### Rules

| Condition | Tracking |
|----------|--------|
| Robot moving | ❌ Disabled |
| Robot stopped < 5s | ❌ Disabled |
| Robot stopped ≥ 5s + allowed | ✅ Enabled |
| Manually disabled | ❌ Disabled |

---

### Logic Summary

```text
moving = true  → tracking OFF
moving = false → start timer
after 5s       → tracking ON (if allowed)
```

---

## 🧱 MQTT Architecture

```text
Frontend
   ↓ HTTP
Backend
   ↓ MQTT
Broker
   ↓ MQTT
ESP32
```

### Important Notes

- Frontend does NOT talk MQTT directly
- ESP32 does NOT implement tracking logic
- Backend is the **decision layer**

---

## 🧭 Topic Structure

```
robot/
├── cmd/
│   └── drive
├── state/
│   └── drive
├── telemetry/
│   ├── fast
│   └── slow
├── tracking/
│   ├── status
│   └── command
├── vision/
│   └── status   # future
```

---

## 🔧 MQTT Debug

### View all robot topics
```bash
mosquitto_sub -h localhost -t "robot/#" -v
```

---

### View tracking only
```bash
mosquitto_sub -h localhost -t "robot/tracking/#" -v
```

---

### Send drive command
```bash
mosquitto_pub -h localhost -t "robot/cmd/drive" -m "100,100"
```

---

### Stop robot
```bash
mosquitto_pub -h localhost -t "robot/cmd/drive" -m "0,0"
```

---

### Enable tracking
```bash
mosquitto_pub -h localhost -t "robot/tracking/command" -m '{"tracking_allowed":true}'
```

---

### Disable tracking
```bash
mosquitto_pub -h localhost -t "robot/tracking/command" -m '{"tracking_allowed":false}'
```

---

## 🎯 Current State

- ✅ Drive control via MQTT
- ✅ Telemetry (fast + slow)
- ✅ Applied drive state with `moving`
- ✅ Tracking state machine (backend)
- ✅ Tracking enable/disable via MQTT + HTTP
- 🔧 Vision integration (next step)

---

## 🚀 Next Steps

- Implement `robot/vision/status`
- Integrate object detection (camera)
- Link tracking to servo + laser
- Add bounding box visualization in frontend

---