"""
MQTT Robot Simulator
- Mirrors ESP32 main.cpp behavior:
  - Subscribes to robot/cmd/drive with payload "L,R"
  - Applies drive command internally (no MQTT ACK publish, only console log)
  - Publishes:
    - robot/telemetry/fast every 200 ms with ToF + IMU (imu=null if invalid)
    - robot/telemetry/slow change-based OR every 30 s max
"""

import os
import json
import time
import random
import threading
from pathlib import Path
import paho.mqtt.client as mqtt


# =========================
# CONFIG / DEFAULTS
# =========================

def load_defaults():
    """
    Load shared defaults from: shared/config/defaults.json
    Works when running from simulator/ folder or repo root.
    """
    try:
        repo_root = Path(__file__).resolve().parents[1]
        defaults_path = repo_root / "shared" / "config" / "defaults.json"
        with defaults_path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}

defaults = load_defaults()

DEFAULT_MQTT_HOST = defaults.get("mqtt", {}).get("host", "127.0.0.1")
DEFAULT_MQTT_PORT = int(defaults.get("mqtt", {}).get("port", 1883))

MQTT_HOST = os.getenv("MQTT_HOST", DEFAULT_MQTT_HOST)
MQTT_PORT = int(os.getenv("MQTT_PORT", str(DEFAULT_MQTT_PORT)))

TOPIC_FAST = os.getenv("MQTT_TOPIC_FAST", "robot/telemetry/fast")
TOPIC_SLOW = os.getenv("MQTT_TOPIC_SLOW", "robot/telemetry/slow")
TOPIC_CMD_DRIVE = os.getenv("MQTT_TOPIC_CMD_DRIVE", "robot/cmd/drive")


# =========================
# ESP32-LIKE TIMINGS
# =========================
BME_INTERVAL_MS = 5000
IMU_INTERVAL_MS = 100
TOF_INTERVAL_MS = 100

FAST_PUB_MS = 200
SLOW_PUB_MAX_MS = 30000

# BME thresholds (ESP32)
BME_DELTA_T = 0.2
BME_DELTA_H = 1.0
BME_DELTA_P = 1.0


# =========================
# STATE (mirrors cached ESP32 values)
# =========================
state_lock = threading.Lock()

# Cached latest sensor values (like lastT, lastPitch, lastFrontLeftMm, etc.)
lastT = float("nan")
lastH = float("nan")
lastP = float("nan")

lastPitch = float("nan")
lastRoll = float("nan")

# ToF raw (ESP32 uses uint16_t with 0xFFFF invalid)
lastFrontLeftMm = 0xFFFF
lastFrontRightMm = 0xFFFF

# Last published BME values (pubT/pubH/pubP)
pubT = float("nan")
pubH = float("nan")
pubP = float("nan")

# Motor command state (represents drive(left,right))
motor_left = 0
motor_right = 0


def clamp_int(v: int, vmin: int, vmax: int) -> int:
    return max(vmin, min(vmax, v))


def clamp_float(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def is_invalid_u16(v: int) -> bool:
    return v == 0xFFFF


def is_no_obj_tof(v: int) -> bool:
    # ESP32: invalid OR >= 8000 means NO OBJ
    return is_invalid_u16(v) or v >= 8000


def fabs(x: float) -> float:
    return -x if x < 0 else x


def should_publish_bme(t: float, h: float, p: float, now_ms: int, last_slow_pub_ms: int) -> bool:
    # Mirrors ESP32 shouldPublishBME()
    global pubT, pubH, pubP

    if (pubT != pubT) or (pubH != pubH) or (pubP != pubP):  # NaN check
        return True

    if fabs(t - pubT) >= BME_DELTA_T:
        return True
    if fabs(h - pubH) >= BME_DELTA_H:
        return True
    if fabs(p - pubP) >= BME_DELTA_P:
        return True

    if now_ms - last_slow_pub_ms >= SLOW_PUB_MAX_MS:
        return True

    return False


def parse_drive_payload(payload: str) -> tuple[int, int]:
    """
    Matches ESP32 mqtt_manager behavior expected by onMqttDriveCommand(left,right)
    Payload format: "L,R"
    Clamped to [-255, 255]
    """
    parts = payload.strip().split(",")
    if len(parts) != 2:
        raise ValueError("Expected format 'L,R'")
    left = clamp_int(int(parts[0].strip()), -255, 255)
    right = clamp_int(int(parts[1].strip()), -255, 255)
    return left, right


# =========================
# MQTT callbacks
# =========================
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print(f"[simulator] Connected to MQTT {MQTT_HOST}:{MQTT_PORT}")
        client.subscribe(TOPIC_CMD_DRIVE)
        print(f"[simulator] Subscribed to: {TOPIC_CMD_DRIVE}")
    else:
        print(f"[simulator] MQTT connection failed, reason_code={reason_code}")


def on_message(client, userdata, msg):
    global motor_left, motor_right
    if msg.topic != TOPIC_CMD_DRIVE:
        return

    try:
        raw = msg.payload.decode("utf-8", errors="replace")
        left, right = parse_drive_payload(raw)

        # Mirrors onMqttDriveCommand(left,right) -> drive(left,right)
        with state_lock:
            motor_left = left
            motor_right = right

        # Mirrors Serial.printf("MQTT CMD -> L=%d R=%d\n", ...)
        print(f"MQTT CMD -> L={left} R={right}")

    except Exception as e:
        print(f"[simulator] Invalid drive command: {e}")


# =========================
# Sensor simulation (timed like ESP32 loop)
# =========================
def simulate_bme_step():
    """Update lastT/lastH/lastP roughly like BME readings."""
    global lastT, lastH, lastP

    # Initialize if NaN
    if lastT != lastT:
        lastT = 21.0
        lastH = 50.0
        lastP = 1000.0
        return

    lastT = clamp_float(lastT + random.uniform(-0.2, 0.2), -10.0, 45.0)
    lastH = clamp_float(lastH + random.uniform(-1.0, 1.0), 0.0, 100.0)
    lastP = clamp_float(lastP + random.uniform(-0.5, 0.5), 950.0, 1050.0)


def simulate_imu_step():
    """Update lastPitch/lastRoll roughly like IMU readings."""
    global lastPitch, lastRoll

    # Occasionally simulate IMU not ready (NaN) like ESP32 imu not OK
    if random.random() < 0.01:
        lastPitch = float("nan")
        lastRoll = float("nan")
        return

    if lastPitch != lastPitch:
        lastPitch = 0.0
        lastRoll = 0.0

    lastPitch = clamp_float(lastPitch + random.uniform(-0.3, 0.3), -30.0, 30.0)
    lastRoll = clamp_float(lastRoll + random.uniform(-0.3, 0.3), -30.0, 30.0)


def simulate_tof_step():
    """Update lastFrontLeftMm/lastFrontRightMm like ToF readings."""
    global lastFrontLeftMm, lastFrontRightMm

    # Start valid-ish
    if is_invalid_u16(lastFrontLeftMm):
        lastFrontLeftMm = 600
    if is_invalid_u16(lastFrontRightMm):
        lastFrontRightMm = 600

    # Random walk
    lastFrontLeftMm = int(clamp_int(lastFrontLeftMm + random.randint(-30, 30), 30, 2000))
    lastFrontRightMm = int(clamp_int(lastFrontRightMm + random.randint(-30, 30), 30, 2000))

    # Occasionally simulate "no object" by forcing >= 8000
    if random.random() < 0.02:
        lastFrontLeftMm = 8000
    if random.random() < 0.02:
        lastFrontRightMm = 8000


def build_fast_payload() -> str:
    """
    Mirrors ESP32 fast payload:
      - fl/fr are -1 if NO OBJ, else mm
      - imu is null if NaN pitch/roll
    """
    fl = -1 if is_no_obj_tof(lastFrontLeftMm) else int(lastFrontLeftMm)
    fr = -1 if is_no_obj_tof(lastFrontRightMm) else int(lastFrontRightMm)

    if (lastPitch != lastPitch) or (lastRoll != lastRoll):
        payload = {"tof": {"fl": fl, "fr": fr}, "imu": None}
    else:
        payload = {"tof": {"fl": fl, "fr": fr}, "imu": {"pitch": round(lastPitch, 2), "roll": round(lastRoll, 2)}}

    return json.dumps(payload)


def build_slow_payload() -> str:
    """Mirrors ESP32 slow payload."""
    payload = {"bme": {"t": round(lastT, 2), "h": round(lastH, 2), "p": round(lastP, 2)}}
    return json.dumps(payload)


# =========================
# Main loop (ESP32-like scheduling)
# =========================
def main():
    global pubT, pubH, pubP

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_HOST, MQTT_PORT, 30)
    client.loop_start()

    last_bme_ms = 0
    last_imu_ms = 0
    last_tof_ms = 0

    last_fast_pub_ms = 0
    last_slow_pub_ms = 0

    print("MQTT robot simulator started")

    try:
        while True:
            now_ms = int(time.time() * 1000)

            with state_lock:
                # Periodic sensor reads (like ESP32 loop timing)
                if now_ms - last_bme_ms >= BME_INTERVAL_MS:
                    last_bme_ms = now_ms
                    simulate_bme_step()

                if now_ms - last_imu_ms >= IMU_INTERVAL_MS:
                    last_imu_ms = now_ms
                    simulate_imu_step()

                if now_ms - last_tof_ms >= TOF_INTERVAL_MS:
                    last_tof_ms = now_ms
                    simulate_tof_step()

                # FAST telemetry publish every 200 ms
                if now_ms - last_fast_pub_ms >= FAST_PUB_MS:
                    last_fast_pub_ms = now_ms
                    client.publish(TOPIC_FAST, build_fast_payload())

                # SLOW telemetry publish change-based or max interval
                if (lastT == lastT) and (lastH == lastH) and (lastP == lastP):
                    if should_publish_bme(lastT, lastH, lastP, now_ms, last_slow_pub_ms):
                        last_slow_pub_ms = now_ms
                        pubT, pubH, pubP = lastT, lastH, lastP
                        client.publish(TOPIC_SLOW, build_slow_payload())

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping simulator.")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
