import time
import requests

from servos.servo_controller import ServoController

DETECTIONS_URL = "http://127.0.0.1:8080/detections"
HTTP_TIMEOUT = 0.25

FRAME_W = 320
FRAME_H = 240
CENTER_X = FRAME_W / 2.0
CENTER_Y = FRAME_H / 2.0

# --------------------
# Tuning
# --------------------
CONF_MIN = 0.55

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


# --------------------
# Main loop
# --------------------
def main():
    sc = ServoController()

    pan = PAN_CENTER
    tilt = TILT_CENTER

    last_center = None
    last_seen_time = 0.0

    cx_s = None
    cy_s = None

    moving_x = False
    moving_y = False

    sc.set(pan, tilt)
    print(f"Start: pan={pan:.1f} tilt={tilt:.1f}")

    try:
        while True:
            now = time.time()
            center = get_detection(last_center=last_center)

            if center is not None:
                cx, cy = center
                last_center = (cx, cy)
                last_seen_time = now
            else:
                if last_center is None or (now - last_seen_time) > TARGET_LOST_TIMEOUT:
                    time.sleep(LOOP_DELAY)
                    continue
                cx, cy = last_center

            if SMOOTHING_ENABLED:
                if cx_s is None:
                    cx_s, cy_s = cx, cy
                else:
                    a = EMA_ALPHA
                    cx_s = a * cx + (1.0 - a) * cx_s
                    cy_s = a * cy + (1.0 - a) * cy_s
                cx_use, cy_use = cx_s, cy_s
            else:
                cx_use, cy_use = cx, cy

            ex, ey = norm_error(cx_use, cy_use)

            if moving_x:
                if abs(ex) < DBX_LOW:
                    moving_x = False
            else:
                if abs(ex) > DBX_HIGH:
                    moving_x = True

            if moving_y:
                if abs(ey) < DBY_LOW:
                    moving_y = False
            else:
                if abs(ey) > DBY_HIGH:
                    moving_y = True

            ex_cmd = ex if moving_x else 0.0
            ey_cmd = ey if moving_y else 0.0

            d_pan = PAN_DIR * (ex_cmd * GAIN_PAN)
            d_tilt = TILT_DIR * (ey_cmd * GAIN_TILT)

            d_pan = clamp_step(d_pan, MAX_STEP_PAN)
            d_tilt = clamp_step(d_tilt, MAX_STEP_TILT)

            pan = clamp(pan + d_pan, PAN_MIN, PAN_MAX)
            tilt = clamp(tilt + d_tilt, TILT_MIN, TILT_MAX)

            sc.set(pan, tilt)

            if DEBUG:
                print(
                    f"cx={cx_use:.1f} cy={cy_use:.1f} "
                    f"ex={ex:+.3f} ey={ey:+.3f} "
                    f"mx={moving_x} my={moving_y} "
                    f"d_pan={d_pan:+.3f} d_tilt={d_tilt:+.3f} "
                    f"pan={pan:.1f} tilt={tilt:.1f}"
                )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        pass
    finally:
        sc.close()
        print("Bye.")


if __name__ == "__main__":
    main()