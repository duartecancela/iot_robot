# MQTT Topics - IoT Robot 4WD

## 📌 Overview

This document describes the MQTT topics used in the IoT Robot project, including:
- Existing topics
- Topics to be implemented (vision and tracking)
- Recommended structure

---

## ✅ Existing Topics

### 🔹 Control (Frontend → Robot)

- **robot/cmd/drive**
  - Sends movement commands (forward, backward, left, right, stop)

- **robot/cmd/drive/ack**
  - Acknowledgment from ESP32

---

### 🔹 Robot State

- **robot/state/drive**
  - Current robot movement state
  - Example:
    ```json
    {
      "moving": true
    }
    ```

---

### 🔹 Telemetry

- **robot/telemetry**
- **robot/telemetry/fast**
- **robot/telemetry/slow**

Contains:
- sensor data
- system state
- performance metrics

---

## 🆕 Topics to Implement (Vision + Tracking)

### 🔹 Vision (Raspberry Pi → Frontend)

- **robot/vision/status**

Used to send object detection data.

**Example:**
```json
{
  "detected": true,
  "label": "cell_phone",
  "confidence": 0.87,
  "timestamp": 1712300000.12
}
```

When no object is detected:
```json
{
  "detected": false,
  "timestamp": 1712300001.45
}
```

---

### 🔹 Tracking (Raspberry Pi → Frontend)

- **robot/tracking/status**

Indicates current tracking state.

**Example:**
```json
{
  "tracking_active": true,
  "tracking_allowed": true,
  "robot_is_moving": false
}
```

---

### 🔹 Tracking Command (Frontend → Raspberry Pi)

- **robot/tracking/command**

Allows manual enable/disable of tracking.

**Example:**
```json
{
  "tracking_allowed": true
}
```

or

```json
{
  "tracking_allowed": false
}
```

---

## 🧠 System Logic

### Tracking Rules

Tracking is only active when:

- The robot is **not moving**
- It has been stopped for at least **5 seconds**
- Tracking is **enabled from the frontend**

### Cases

| Condition | Tracking |
|----------|--------|
| Robot moving | ❌ Disabled |
| Robot stopped < 5s | ❌ Disabled |
| Robot stopped ≥ 5s + allowed | ✅ Enabled |
| Manually disabled | ❌ Disabled |

---

## 🖥️ Frontend Behavior

The frontend should:

### Display:
- Video stream
- Detection status:
  - Object detected / not detected
- Tracking status:
  - Active / Inactive
- Robot state:
  - Moving / Stopped

### Provide:
- Button to enable/disable tracking

---

## 🧱 Recommended MQTT Structure

```text
robot/
├── cmd/
│   └── drive
├── state/
│   └── drive
├── telemetry/
│   ├── fast
│   └── slow
├── vision/
│   └── status
├── tracking/
│   ├── status
│   └── command
```

---

## 🔧 MQTT Debug

View all topics:

```bash
mosquitto_sub -h localhost -t "#" -v
```

Filter robot topics:

```bash
mosquitto_sub -h localhost -t "robot/#" -v
```

---

## 🎯 Current Goal

Implement a simple system:

- Object detection (cell_phone)
- MQTT publishing
- Frontend visualization
- Tracking only when:
  - robot stopped ≥ 5 seconds
  - tracking enabled

---

## 🚀 Next Steps

- [ ] Implement `robot/vision/status`
- [ ] Implement `robot/tracking/status`
- [ ] Implement `robot/tracking/command`
- [ ] Integrate MQTT in frontend
- [ ] Add tracking control button
- [ ] Display states in UI

---