# Robot MQTT Simulator (Python)

This Python-based simulator emulates the **ESP32 robot behavior**, publishing MQTT telemetry
using **the same topics and payload formats** as the real robot.

Its purpose is to allow development and testing of the **frontend and MQTT logic**
without requiring the physical robot to be powered on.

---

## Requirements

- Python **3.10 or newer**
- A running MQTT broker (e.g. Mosquitto)
- Git Bash (or any compatible terminal)
- MQTT Explorer (for visualization)

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

---

## 2) Activate the virtual environment

### Git Bash (Windows)

```bash
source .venv/Scripts/activate
```

When active, your terminal prompt will show:

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

Open `main.py` and confirm the broker address:

```python
MQTT_HOST = "192.168.1.110"
MQTT_PORT = 1883
```

---

## 5) Run the simulator

```bash
python main.py
```

Expected output:

```
MQTT robot simulator started
```

The simulator will now run continuously.

---

## 6) Visualize data with MQTT Explorer

In **MQTT Explorer**:

1. Connect to the broker (`192.168.1.110`)
2. Open the topic tree:
   ```
   robot
     └─ telemetry
         ├─ fast
         └─ slow
   ```

### Fast telemetry (`robot/telemetry/fast`)
Updated approximately every **100 ms**:

```json
{
  "tof": { "fl": 643, "fr": 621 },
  "imu": { "pitch": 5.39, "roll": -10.51 }
}
```

### Slow telemetry (`robot/telemetry/slow`)
Updated every **5 seconds**:

```json
{
  "bme": { "t": 21.35, "h": 52.28, "p": 992.64 }
}
```

Note: in MQTT Explorer, the `slow` topic may appear empty until the **first message is published**.

---

## 7) Stop the simulator

In the terminal where the simulator is running:

```
CTRL + C
```

---

## Notes

- The simulator publishes **exactly the same topics and payloads** as the real ESP32
- It can be used as a *drop-in replacement* for the robot during development
- At this stage, the simulator **only publishes telemetry** (no command handling yet)

---

## Planned next steps

- Receive MQTT commands (motors, pan/tilt, laser)
- Simulate robot physical reactions
- Use the simulator as a full backend for the web frontend
