# ESP32 4WD IoT Pest Deterrent Robot

This project implements a **4WD IoT robot** based on **ESP32 + Raspberry Pi**, designed for **remote control and pest deterrence** in outdoor or rural environments.

The system uses a **distributed architecture**:
- **ESP32** → low-level control (motors, sensors, actuators)
- **Raspberry Pi** → high-level control, vision, web interface and MQTT coordination

Communication between components is performed via **MQTT**.

---

## System Overview

The robot is designed to:
- Be **remotely controlled via MQTT**
- Stream sensor telemetry efficiently
- Use **computer vision** to detect pests
- Actively deter pests using:
  - a **camera mounted on a pan/tilt gimbal**
  - a **laser pointer** aligned with the camera
  - optional sound/light deterrents (future work)

The Raspberry Pi acts as the **central controller**, hosting:
- an MQTT broker or client
- a **web-based control interface**
- camera processing and decision logic

---

## Architecture

```
Web Interface
     │
     ▼
Raspberry Pi (MQTT client / server)
  ├─ Vision processing (camera)
  ├─ Pest detection logic
  ├─ Servo + laser commands
  └─ Telemetry consumer
     │
     ▼
ESP32 (MQTT client)
  ├─ Motor control (4WD)
  ├─ Sensor acquisition
  ├─ Servo and laser actuation
  └─ Telemetry publisher
```

---

## ESP32 Responsibilities

The ESP32 is responsible for **real-time and safety-critical tasks**:

- Motor control (**Bluetooth and MQTT commands**)
- Sensor acquisition:
  - **VL53L0X (ToF)** – front-left and front-right obstacle detection
  - **IMU (MPU6050)** – pitch and roll
  - **BME280** – temperature, humidity, pressure
- Actuation:
  - servo motors (camera pan/tilt)
  - laser module (on/off, aiming)
- MQTT telemetry publishing
- **MQTT remote drive command reception (RX) with ACK confirmation**

The ESP32 continues operating **independently** even if Wi-Fi or MQTT connectivity is temporarily lost.

---

## Project Structure

```
ESP32_BTS_Driver/
├── .vscode/                 # VS Code configuration
├── include/                 # Header files (.h)
├── lib/                     # Local libraries (if any)
├── src/
│   ├── bluetooth_control.cpp  # Bluetooth motor control
│   ├── bme280_sensor.cpp      # BME280 environmental sensor
│   ├── imu_sensor.cpp         # IMU (MPU6050)
│   ├── main.cpp               # Main loop and system integration
│   ├── motor_control.cpp      # Motor drivers and logic
│   ├── mqtt_manager.cpp       # MQTT client (TX + RX + ACK)
│   ├── vl53l0x_sensor.cpp     # ToF sensors (front-left / front-right)
│   └── wifi_manager.cpp       # Wi-Fi management (WiFiMulti)
├── test/                     # Tests (optional)
├── .gitignore
├── platformio.ini
├── secrets.example.ini       # Example configuration
├── secrets.ini               # Private configuration (not committed)
└── README.md
```

---

## Sensor Reading Strategy

### Sensor acquisition rates
- **ToF (VL53L0X)**: fast (≈ 100 ms)
- **IMU (MPU6050)**: fast (≈ 100 ms)
- **BME280**: slow (≈ 5 s)

### Telemetry philosophy
- Fast sensors are used for **local control and safety**
- Environmental data is treated as **low-frequency telemetry**
- MQTT traffic is optimized to avoid unnecessary network load

---

## MQTT Telemetry Topics

### 1) Fast telemetry (navigation and stability)
**Topic**
```
robot/telemetry/fast
```

**Content**
- ToF distances (front-left, front-right)
- IMU pitch and roll

**Example payload**
```json
{
  "tof": { "fl": 245, "fr": 90 },
  "imu": { "pitch": 5.23, "roll": -0.96 }
}
```

---

### 2) Slow telemetry (environmental data)
**Topic**
```
robot/telemetry/slow
```

**Content**
- Temperature
- Humidity
- Pressure

**Example payload**
```json
{
  "bme": { "t": 21.35, "h": 52.28, "p": 992.64 }
}
```

---

## MQTT Remote Drive Control (RX)

The ESP32 supports **remote motor control via MQTT**, in addition to Bluetooth-based local control.

### Command and ACK topics
- **Drive command topic:**  
  ```
  robot/cmd/drive
  ```
- **ACK (acknowledgement) topic:**  
  ```
  robot/cmd/drive/ack
  ```

The ACK mechanism provides **positive confirmation** that the ESP32:
1. Received the MQTT message  
2. Parsed the payload correctly  
3. Applied the motor command  

---

### Supported drive command payloads

#### 1) CSV format (recommended for quick CLI testing)
```
120,120
-120,120
0,0
```

#### 2) JSON format (recommended for backend / frontend integration)
```json
{"left":120,"right":120}
```

Motor values are automatically clamped to the range `[-255, 255]`.

---

### ACK payload examples

**Successful command**
```json
{"ok":true,"left":120,"right":120}
```

**Invalid payload**
```json
{"ok":false,"err":"invalid_payload"}
```

---

## CLI Testing (MQTT)

Telemetry and commands can be tested from any machine with Mosquitto tools installed.

> ⚠️ On Windows, **Git Bash is strongly recommended**.  
> PowerShell may alter JSON quoting and cause invalid payloads.

### Subscribe to drive ACK
```bash
mosquitto_sub -h <BROKER_IP> -t "robot/cmd/drive/ack" -v
```

### Send drive commands (CSV)
```bash
mosquitto_pub -h <BROKER_IP> -t robot/cmd/drive -m "120,120"
mosquitto_pub -h <BROKER_IP> -t robot/cmd/drive -m "-120,120"
mosquitto_pub -h <BROKER_IP> -t robot/cmd/drive -m "0,0"
```

### Send drive command (JSON)
```bash
mosquitto_pub -h <BROKER_IP> -t robot/cmd/drive -m '{"left":120,"right":120}'
```

---

## Monitoring MQTT Traffic

Telemetry can be monitored from any machine with Mosquitto tools installed.

```bash
mosquitto_sub -h <BROKER_IP> -t 'robot/telemetry/#' -v
```

Example:
```bash
mosquitto_sub -h 192.168.1.110 -t 'robot/telemetry/#' -v
```

---

## `secrets.ini` Configuration

The file `secrets.ini` contains **private credentials** and must **not** be committed to the repository.

### 1) Create the configuration file
```bash
cp secrets.example.ini secrets.ini
```

### 2) Example content (`secrets.ini`)
```ini
[env]
WIFI_SSID_PRIMARY = "YOUR_WIFI_SSID"
WIFI_PASS_PRIMARY = "YOUR_WIFI_PASSWORD"

WIFI_SSID_SECONDARY = "BACKUP_NETWORK"
WIFI_PASS_SECONDARY = "BACKUP_PASSWORD"

MQTT_HOST="BROKER_IP_ADDRESS"
MQTT_PORT=1883
```

This approach:
- separates credentials from source code
- allows easy deployment to different environments
- keeps the repository secure

---

## Control and Vision (Raspberry Pi)

The Raspberry Pi is responsible for:
- Hosting the **web-based control interface**
- Running **computer vision algorithms**
- Detecting pests using the camera feed
- Sending control commands to the ESP32 via MQTT:
  - movement
  - camera pan/tilt
  - laser activation

The camera is mounted on a **pan/tilt gimbal**, with the laser aligned to the camera’s optical axis.

---

## Design Principles

- Local control and safety first
- MQTT used for coordination and telemetry
- Clear separation between low-level and high-level logic
- Modular and extensible design
- Suitable for research, prototyping and field deployment

---

## Current Status

- ✅ Motor control operational (Bluetooth + MQTT)
- ✅ MQTT telemetry publishing (fast and slow topics)
- ✅ MQTT remote drive control with ACK confirmation
- ✅ ToF, IMU and BME280 sensors working
- ✅ Wi-Fi with fallback networks
- 🔧 Vision-based pest detection under development
- 🔧 Web interface and laser targeting in progress

---
