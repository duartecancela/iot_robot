import json
import os
import time
import threading
import requests

import paho.mqtt.client as mqtt

from servos.servo_controller import ServoController
from laser.laser_controller import fire

DETECTIONS_URL = "http://127.0.0.1:8080/detections"
HTTP_TIMEOUT = 0.25

FRAME_W = 320
FRAME_H = 240
CENTER_X = FRAME_W / 2.0
CENTER_Y = FRAME_H / 2.0

# --------------------
# MQTT
# --------------------
MQTT_BROKER_HOST = os.getenv("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
MQTT_TOPIC_TRACKING_STATUS = "robot/tracking/status"

tracking_allowed = True
tracking_active = False
robot_is_moving = False

state_lock = threading.Lock()

# --------------------
# Tuning
# --------------------
CONF_MIN = 0.15

DBX_LOW = 0.10
DBX_HIGH = 0.16

DBY_LOW = 0.12
DBY_HIGH = 0.20

GAIN_PAN = 6.0
GAIN_TILT = 4.0

MAX_STEP_PAN = 0.8
MAX_STEP_TILT = 0.5

LOOP_DELAY = 0.06

PAN_DIR = -1
TILT_DIR = +1

DEBUG = True

# --------------------
# Smoothing
# --------------------
SMOOTHING_ENABLED = True
EMA_ALPHA = 0.12

# --------------------
# Mechanical limits
# --------------------
PAN_CENTER = 90.0
TILT_CENTER = 120.0

PAN_MIN = 20.0
PAN_MAX = 160.0

TILT_MIN = 30.0
TILT_MAX = 140.0

# --------------------
# Target loss handling
# --------------------
TARGET_LOST_TIMEOUT = 0.50

# --------------------
# Laser firing logic
# --------------------
CENTER_HOLD_TIME = 0.6
LASER_COOLDOWN = 4.0
CENTER_FIRE_X = 0.14
CENTER_FIRE_Y = 0.18

# --------------------
# Helpers
# --------------------
def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))


def clamp_step(delta, max_step):
    if delta > max_step:
        return max_step
    if delta < -max_step:
        return -max_step
    return delta


def norm_error(px, py):
    ex = (px - CENTER_X) / CENTER_X
    ey = (py - CENTER_Y) / CENTER_Y
    return ex, ey


def extract_center(det):
    if not isinstance(det, dict):
        return None

    for kx, ky in [("cx", "cy"), ("center_x", "center_y")]:
        if kx in det and ky in det:
            try:
                return float(det[kx]), float(det[ky])
            except Exception:
                pass

    if all(k in det for k in ("x", "y", "w", "h")):
        try:
            x = float(det["x"])
            y = float(det["y"])
            w = float(det["w"])
            h = float(det["h"])
            return x + w / 2.0, y + h / 2.0
        except Exception:
            return None

    return None


def det_conf(det):
    try:
        return float(det.get("conf", det.get("confidence", 0.0)))
    except Exception:
        return 0.0


def pick_best_detection(dets, last_center=None):
    if not isinstance(dets, list) or not dets:
        return None

    if last_center is not None:
        lx, ly = last_center

        def score(d):
            c = extract_center(d)
            if not c:
                return -1e18
            cx, cy = c
            conf = det_conf(d)
            dx = cx - lx
            dy = cy - ly
            dist2 = dx * dx + dy * dy
            return conf * 1000.0 - dist2

        return max(dets, key=score)

    return max(dets, key=det_conf)


def get_detection(last_center=None):
    try:
        r = requests.get(DETECTIONS_URL, timeout=HTTP_TIMEOUT)
        if r.status_code != 200:
            return None

        data = r.json()
        dets = data["detections"] if isinstance(data, dict) and "detections" in data else data

        det = pick_best_detection(dets, last_center=last_center)
        if not det or det_conf(det) < CONF_MIN:
            return None

        return extract_center(det)
    except Exception:
        return None


def fire_laser_async(state):
    try:
        fire()
    finally:
        state["laser_active"] = False


def get_tracking_state():
    with state_lock:
        return tracking_allowed, tracking_active, robot_is_moving


def on_connect(client, userdata, flags, rc, properties=None):
    if DEBUG:
        print(f"[MQTT] Connected with rc={rc}")
    client.subscribe(MQTT_TOPIC_TRACKING_STATUS)


def on_message(client, userdata, msg):
    global tracking_allowed, tracking_active, robot_is_moving

    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except Exception as e:
        print(f"[MQTT] Invalid JSON on {msg.topic}: {e}")
        return

    with state_lock:
        tracking_allowed = bool(data.get("tracking_allowed", tracking_allowed))
        tracking_active = bool(data.get("tracking_active", tracking_active))
        robot_is_moving = bool(data.get("robot_is_moving", robot_is_moving))

    if DEBUG:
        print(
            "[MQTT] tracking status updated -> "
            f"tracking_allowed={tracking_allowed} "
            f"tracking_active={tracking_active} "
            f"robot_is_moving={robot_is_moving}"
        )


def start_mqtt_client():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
    client.loop_start()
    return client


def reset_runtime_state():
    return {
        "last_center": None,
        "last_seen_time": 0.0,
        "cx_s": None,
        "cy_s": None,
        "moving_x": False,
        "moving_y": False,
        "stable_since": None,
        "fired_for_current_lock": False,
    }


# --------------------
# Main loop
# --------------------
def main():
    sc = ServoController()
    mqtt_client = start_mqtt_client()

    pan = PAN_CENTER
    tilt = TILT_CENTER

    runtime = reset_runtime_state()

    last_fire_time = 0.0
    laser_state = {"laser_active": False}

    last_tracking_active = None

    sc.set(pan, tilt)
    print(f"Start: pan={pan:.1f} tilt={tilt:.1f}")

    try:
        while True:
            now = time.time()

            current_tracking_allowed, current_tracking_active, current_robot_is_moving = get_tracking_state()

            if current_tracking_active != last_tracking_active:
                print(
                    "[STATE] "
                    f"tracking_allowed={current_tracking_allowed} "
                    f"tracking_active={current_tracking_active} "
                    f"robot_is_moving={current_robot_is_moving}"
                )

                pan = PAN_CENTER
                tilt = TILT_CENTER
                sc.set(pan, tilt)

                runtime = reset_runtime_state()
                last_tracking_active = current_tracking_active

            if not current_tracking_active:
                time.sleep(LOOP_DELAY)
                continue

            center = get_detection(last_center=runtime["last_center"])

            if center is not None:
                cx, cy = center
                runtime["last_center"] = (cx, cy)
                runtime["last_seen_time"] = now
            else:
                if runtime["last_center"] is None or (now - runtime["last_seen_time"]) > TARGET_LOST_TIMEOUT:
                    runtime["stable_since"] = None
                    runtime["fired_for_current_lock"] = False
                    time.sleep(LOOP_DELAY)
                    continue
                cx, cy = runtime["last_center"]

            if SMOOTHING_ENABLED:
                if runtime["cx_s"] is None:
                    runtime["cx_s"], runtime["cy_s"] = cx, cy
                else:
                    a = EMA_ALPHA
                    runtime["cx_s"] = a * cx + (1.0 - a) * runtime["cx_s"]
                    runtime["cy_s"] = a * cy + (1.0 - a) * runtime["cy_s"]
                cx_use, cy_use = runtime["cx_s"], runtime["cy_s"]
            else:
                cx_use, cy_use = cx, cy

            ex, ey = norm_error(cx_use, cy_use)

            if runtime["moving_x"]:
                if abs(ex) < DBX_LOW:
                    runtime["moving_x"] = False
            else:
                if abs(ex) > DBX_HIGH:
                    runtime["moving_x"] = True

            if runtime["moving_y"]:
                if abs(ey) < DBY_LOW:
                    runtime["moving_y"] = False
            else:
                if abs(ey) > DBY_HIGH:
                    runtime["moving_y"] = True

            ex_cmd = ex if runtime["moving_x"] else 0.0
            ey_cmd = ey if runtime["moving_y"] else 0.0

            d_pan = PAN_DIR * (ex_cmd * GAIN_PAN)
            d_tilt = TILT_DIR * (ey_cmd * GAIN_TILT)

            d_pan = clamp_step(d_pan, MAX_STEP_PAN)
            d_tilt = clamp_step(d_tilt, MAX_STEP_TILT)

            pan = clamp(pan + d_pan, PAN_MIN, PAN_MAX)
            tilt = clamp(tilt + d_tilt, TILT_MIN, TILT_MAX)

            sc.set(pan, tilt)

            centered_for_fire = (
                abs(ex) < CENTER_FIRE_X and
                abs(ey) < CENTER_FIRE_Y
            )

            if centered_for_fire:
                if runtime["stable_since"] is None:
                    runtime["stable_since"] = now
            else:
                runtime["stable_since"] = None
                runtime["fired_for_current_lock"] = False

            ready_to_fire = (
                runtime["stable_since"] is not None and
                (now - runtime["stable_since"]) >= CENTER_HOLD_TIME and
                not runtime["fired_for_current_lock"] and
                not laser_state["laser_active"] and
                (now - last_fire_time) >= LASER_COOLDOWN
            )

            if ready_to_fire:
                laser_state["laser_active"] = True
                runtime["fired_for_current_lock"] = True
                last_fire_time = now

                threading.Thread(
                    target=fire_laser_async,
                    args=(laser_state,),
                    daemon=True,
                ).start()

            if DEBUG:
                hold_time = 0.0 if runtime["stable_since"] is None else (now - runtime["stable_since"])
                print(
                    f"cx={cx_use:.1f} cy={cy_use:.1f} "
                    f"ex={ex:+.3f} ey={ey:+.3f} "
                    f"mx={runtime['moving_x']} my={runtime['moving_y']} "
                    f"d_pan={d_pan:+.3f} d_tilt={d_tilt:+.3f} "
                    f"pan={pan:.1f} tilt={tilt:.1f} "
                    f"centered={centered_for_fire} hold={hold_time:.2f}s "
                    f"laser_active={laser_state['laser_active']}"
                )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        pass
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        sc.close()
        print("Bye.")


if __name__ == "__main__":
    main()