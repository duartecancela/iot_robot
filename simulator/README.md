# Robot MQTT Simulator (Python)

This Python-based simulator emulates the behavior of the **ESP32-based robot**,
using **the same MQTT topics and payload formats** as the real robot.

Its purpose is to allow development and testing of the **frontend** and MQTT-based
logic **without requiring the physical robot to be powered on**.

---

## Requirements

- Python **3.10 or newer**
- A running MQTT broker (e.g. Mosquitto)
- A terminal (Git Bash, PowerShell, Linux, or macOS)
- MQTT Explorer (optional but recommended)

---

## Structure

```
simulator/
├─ main.py
└─ .venv/        # virtual environment (not committed)
```

---

## 1) Create a virtual environment

From inside the `simulator` directory:

```bash
python -m venv .venv
```

> On some systems you may need to use `python3`.

---

## 2) Activate the virtual environment

### Git Bash (Windows)

```bash
source .venv/Scripts/activate
```

When active, your terminal prompt will look like:

```
(.venv)
```

---

## 3) Install dependencies

```bash
pip install paho-mqtt
```

---

## 4) Configure the MQTT broker

The simulator **does not hardcode any IP address**.

It connects to the MQTT broker using **environment variables** or shared defaults.

### Option A — Environment variables (recommended)

#### Linux / macOS / Git Bash
```bash
export MQTT_HOST=<MQTT_BROKER_IP>
export MQTT_PORT=1883
```

#### Windows PowerShell
```powershell
$env:MQTT_HOST="<MQTT_BROKER_IP>"
$env:MQTT_PORT="1883"
```

#### Windows CMD
```cmd
set MQTT_HOST=<MQTT_BROKER_IP>
set MQTT_PORT=1883
```

---

### Option B — Shared defaults (fallback)

If no environment variables are set, the simulator uses:

```json
"mqtt": {
  "host": "127.0.0.1",
  "port": 1883
}
```

from:

```
shared/config/defaults.json
```

This makes the simulator portable across machines without code changes.

---

## 5) Run the simulator

```bash
python main.py
```

Expected output:

```
MQTT robot simulator started
```

The simulator will now run continuously until stopped with **CTRL + C**.

---

## 6) Supported MQTT topics

### 6.1 Telemetry (same as ESP32)

#### 🔹 Fast telemetry — `robot/telemetry/fast`

Published approximately every **200 ms**.

Example payload:

```json
{
  "tof": { "fl": 643, "fr": 621 },
  "imu": { "pitch": 5.39, "roll": -10.51 }
}
```

Notes:
- `fl` / `fr` are set to `-1` when no obstacle is detected (NO OBJ),
  matching ESP32 behavior.
- `imu` may be `null` when the IMU is considered invalid.

---

#### 🔹 Slow telemetry — `robot/telemetry/slow`

Published when relevant changes occur or, at most, every **30 seconds**.

Example payload:

```json
{
  "bme": { "t": 21.35, "h": 52.28, "p": 992.64 }
}
```

> In MQTT Explorer, this topic may appear empty until the first publication.

---

## 6.2 Drive commands and feedback

### ▶️ Send drive command — `robot/cmd/drive`

The simulator subscribes to this topic and applies the values internally.

#### CSV format
```text
"L,R"
```

Examples:

```bash
mosquitto_pub -h <MQTT_BROKER_IP> -t robot/cmd/drive -m "120,120"
mosquitto_pub -h <MQTT_BROKER_IP> -t robot/cmd/drive -m "-120,120"
mosquitto_pub -h <MQTT_BROKER_IP> -t robot/cmd/drive -m "0,0"
```

#### JSON format (optional)

```json
{"left":120,"right":-80}
```

```bash
mosquitto_pub -h <MQTT_BROKER_IP> -t robot/cmd/drive -m '{"left":120,"right":-80}'
```

Values are automatically clamped to:

```
-255 ≤ left/right ≤ 255
```

---

### ✅ Applied drive state (retained) — `robot/state/drive`

Whenever a valid command is applied, the simulator publishes the current drive state:

- **Topic:** `robot/state/drive`
- **Payload:** `{"left":L,"right":R}`
- **Retained:** `true`

Subscribe with:

```bash
mosquitto_sub -h <MQTT_BROKER_IP> -t robot/state/drive -v
```

This allows any client (frontend/backend) to immediately know the
**current applied motor state**, even after reconnecting.

---

### 🔁 Command ACK (not retained) — `robot/cmd/drive/ack`

For every received command, the simulator sends an acknowledgment:

- **Topic:** `robot/cmd/drive/ack`
- **Retained:** `false`

Success example:

```json
{"ok":true,"left":120,"right":120}
```

Error example:

```json
{"ok":false,"err":"invalid_payload"}
```

Subscribe with:

```bash
mosquitto_sub -h <MQTT_BROKER_IP> -t robot/cmd/drive/ack -v
```

This topic is intended for **confirmation and debugging**, not state storage.

---

## 7) Visualizing with MQTT Explorer

When connected to the broker, the topic tree should look like:

```
robot
 ├─ telemetry
 │   ├─ fast
 │   └─ slow
 ├─ cmd
 │   └─ drive
 │       └─ ack
 └─ state
     └─ drive
```

---

## 8) Stop the simulator

In the terminal where it is running:

```text
CTRL + C
```

---

## Notes

- The simulator mirrors the **MQTT behavior of the ESP32**.
- It can be used as a **drop-in replacement** for the physical robot during development.
- Frontend and backend code can switch between simulator and real robot
  **without any changes**.
- `robot/state/drive` represents **state** (retained).
- `robot/cmd/drive/ack` represents **events** (non-retained).

---

## Possible next steps

- Add MQTT commands for camera pan/tilt
- Add laser control via MQTT
- Simulate physical reactions (IMU / ToF influenced by movement)
- Direct integration with the React frontend
