"""
MQTT Robot Simulator
- Matches ESP32 telemetry structure exactly
- Publishes fast and slow telemetry on separate topics
"""

import time
import json
import random
import paho.mqtt.client as mqtt
import os
from pathlib import Path



# =========================
# MQTT CONFIG
# Resolution order:
# 1) Environment variables
# 2) shared/config/defaults.json
# 3) Internal fallback values
# =========================

def load_defaults():
    """
    Load shared defaults from: shared/config/defaults.json
    Works when running from simulator/ folder or repo root.
    """
    try:
        # simulator/main.py -> repo root is parent of "simulator"
        repo_root = Path(__file__).resolve().parents[1]
        defaults_path = repo_root / "shared" / "config" / "defaults.json"
        with defaults_path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"[simulator] Could not load shared defaults.json: {e}")
        return {}

defaults = load_defaults()

DEFAULT_MQTT_HOST = defaults.get("mqtt", {}).get("host", "127.0.0.1")
DEFAULT_MQTT_PORT = int(defaults.get("mqtt", {}).get("port", 1883))

# Allow overriding via env (useful when switching PC / Raspberry Pi)
MQTT_HOST = os.getenv("MQTT_HOST", DEFAULT_MQTT_HOST)
MQTT_PORT = int(os.getenv("MQTT_PORT", str(DEFAULT_MQTT_PORT)))

# Topics: keep the same structure as backend subscriptions
TOPIC_FAST = os.getenv("MQTT_TOPIC_FAST", defaults.get("mqtt", {}).get("topicTelemetry", "robot/telemetry") + "/fast")
TOPIC_SLOW = os.getenv("MQTT_TOPIC_SLOW", defaults.get("mqtt", {}).get("topicTelemetry", "robot/telemetry") + "/slow")



# =========================
# TIMING (seconds)
# =========================
FAST_INTERVAL = 0.1    # ~100 ms (ToF + IMU)
SLOW_INTERVAL = 5.0    # ~5 s (BME280)


# =========================
# INITIAL SENSOR VALUES
# =========================
tof_fl = 600
tof_fr = 600

imu_pitch = 0.0
imu_roll = 0.0

bme_temp = 21.0
bme_hum = 50.0
bme_press = 1000.0


def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))


def simulate_fast():
    global tof_fl, tof_fr, imu_pitch, imu_roll

    tof_fl += random.randint(-30, 30)
    tof_fr += random.randint(-30, 30)

    imu_pitch += random.uniform(-0.3, 0.3)
    imu_roll += random.uniform(-0.3, 0.3)

    tof_fl = clamp(tof_fl, 30, 2000)
    tof_fr = clamp(tof_fr, 30, 2000)
    imu_pitch = clamp(imu_pitch, -30.0, 30.0)
    imu_roll = clamp(imu_roll, -30.0, 30.0)

    payload = {
        "tof": {
            "fl": int(tof_fl),
            "fr": int(tof_fr)
        },
        "imu": {
            "pitch": round(imu_pitch, 2),
            "roll": round(imu_roll, 2)
        }
    }

    return payload


def simulate_slow():
    global bme_temp, bme_hum, bme_press

    bme_temp += random.uniform(-0.2, 0.2)
    bme_hum += random.uniform(-1.0, 1.0)
    bme_press += random.uniform(-0.5, 0.5)

    bme_temp = clamp(bme_temp, -10.0, 45.0)
    bme_hum = clamp(bme_hum, 0.0, 100.0)
    bme_press = clamp(bme_press, 950.0, 1050.0)

    payload = {
        "bme": {
            "t": round(bme_temp, 2),
            "h": round(bme_hum, 2),
            "p": round(bme_press, 2)
        }
    }

    return payload


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_HOST, MQTT_PORT, 30)
    client.loop_start()

    last_fast = 0
    last_slow = 0

    print("MQTT robot simulator started")

    try:
        while True:
            now = time.time()

            if now - last_fast >= FAST_INTERVAL:
                fast_payload = simulate_fast()
                client.publish(TOPIC_FAST, json.dumps(fast_payload))
                last_fast = now

            if now - last_slow >= SLOW_INTERVAL:
                slow_payload = simulate_slow()
                client.publish(TOPIC_SLOW, json.dumps(slow_payload))
                last_slow = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping simulator...")

    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
