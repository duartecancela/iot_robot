import time
import requests

from servo_controller import ServoController

# --------------------
# Sources
# --------------------
DETECTIONS_URL = "http://127.0.0.1:8080/detections"
HTTP_TIMEOUT = 0.25

# Frame geometry (must match what your detections use)
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W / 2.0
CENTER_Y = FRAME_H / 2.0

# --------------------
# Tuning
# --------------------
CONF_MIN = 0.50          # ignore low-confidence detections
DEADBAND_X = 0.06        # normalized [-1..1] deadband (pan)
DEADBAND_Y = 0.06        # normalized [-1..1] deadband (tilt)

GAIN_PAN = 18.0          # degrees per normalized error (try 12..25)
GAIN_TILT = 18.0         # degrees per normalized error (try 12..25)

MAX_STEP_DEG = 2.0       # clamp per-loop motion (avoid jerks)
LOOP_DELAY = 0.05        # 20 Hz

# If motion is inverted, flip these (+1 or -1)
PAN_DIR = +1             # set to -1 if pan moves wrong direction
TILT_DIR = +1            # set to -1 if tilt moves wrong direction

# --------------------
# Mechanical limits
# --------------------
PAN_CENTER = 90.0
TILT_CENTER = 120.0

PAN_MIN, PAN_MAX = 20.0, 160.0

# IMPORTANT: You said you can only go DOWN 20 degrees from center
# (center=120 -> max down=140).
TILT_MAX = 140.0

# You can go UP much more. Start safe and reduce if you still have margin.
TILT_MIN = 30.0          # <-- adjust: smaller = more up (but confirm no collision)

# --------------------
# Helpers
# --------------------
def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def clamp_step(delta, max_step):
    if delta > max_step:
        return max_step
    if delta < -max_step:
        return -max_step
    return delta

def norm_error(px, py):
    # normalize error to roughly [-1..1]
    ex = (px - CENTER_X) / CENTER_X
    ey = (py - CENTER_Y) / CENTER_Y
    return ex, ey

def pick_best_detection(dets):
    if not isinstance(dets, list) or not dets:
        return None
    def conf(d):
        try:
            return float(d.get("conf", d.get("confidence", 0.0)))
        except Exception:
            return 0.0
    return max(dets, key=conf)

def extract_center(det):
    """
    Tries multiple common formats:
    - cx, cy
    - center: {x,y}
    - x,y,w,h (bbox)
    - bbox: [x1,y1,x2,y2] or {x1,y1,x2,y2}
    - box: {x1,y1,x2,y2} or {x,y,w,h}
    Returns (cx, cy) in pixel coords or None.
    """
    if not isinstance(det, dict):
        return None

    # 1) direct center
    for kx, ky in [("cx", "cy"), ("center_x", "center_y"), ("x_center", "y_center")]:
        if kx in det and ky in det:
            try:
                return float(det[kx]), float(det[ky])
            except Exception:
                pass

    # 2) nested center
    c = det.get("center")
    if isinstance(c, dict) and "x" in c and "y" in c:
        try:
            return float(c["x"]), float(c["y"])
        except Exception:
            pass

    # 3) bbox as x,y,w,h
    for keys in [("x", "y", "w", "h"), ("left", "top", "width", "height")]:
        if all(k in det for k in keys):
            try:
                x, y, w, h = (float(det[keys[0]]), float(det[keys[1]]),
                              float(det[keys[2]]), float(det[keys[3]]))
                return x + w / 2.0, y + h / 2.0
            except Exception:
                pass

    # 4) bbox array [x1,y1,x2,y2]
    bbox = det.get("bbox") or det.get("box")
    if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
        try:
            x1, y1, x2, y2 = map(float, bbox)
            # If it's actually x,y,w,h by mistake (x2 smaller than x1), fallback not possible.
            return (x1 + x2) / 2.0, (y1 + y2) / 2.0
        except Exception:
            pass

    # 5) bbox dict {x1,y1,x2,y2} or {x,y,w,h}
    if isinstance(bbox, dict):
        if all(k in bbox for k in ("x1", "y1", "x2", "y2")):
            try:
                x1, y1, x2, y2 = map(float, (bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]))
                return (x1 + x2) / 2.0, (y1 + y2) / 2.0
            except Exception:
                pass
        if all(k in bbox for k in ("x", "y", "w", "h")):
            try:
                x, y, w, h = map(float, (bbox["x"], bbox["y"], bbox["w"], bbox["h"]))
                return x + w / 2.0, y + h / 2.0
            except Exception:
                pass

    return None

def get_detection():
    try:
        r = requests.get(DETECTIONS_URL, timeout=HTTP_TIMEOUT)
        if r.status_code != 200:
            return None
        data = r.json()
        # allow either list or {detections:[...]}
        if isinstance(data, dict) and "detections" in data:
            data = data["detections"]
        det = pick_best_detection(data)
        if not det:
            return None
        # confidence gate
        try:
            conf = float(det.get("conf", det.get("confidence", 0.0)))
        except Exception:
            conf = 0.0
        if conf < CONF_MIN:
            return None
        center = extract_center(det)
        if not center:
            return None
        return center  # (cx, cy)
    except Exception:
        return None

# --------------------
# Main loop
# --------------------
def main():
    sc = ServoController()

    pan = PAN_CENTER
    tilt = TILT_CENTER

    # start at calibrated center
    sc.set(pan, tilt)
    print(f"Start: pan={pan:.1f} tilt={tilt:.1f} (center)")

    try:
        while True:
            center = get_detection()

            if center is None:
                # no detection -> hold position
                time.sleep(LOOP_DELAY)
                continue

            cx, cy = center
            ex, ey = norm_error(cx, cy)

            # deadband
            if abs(ex) < DEADBAND_X:
                ex = 0.0
            if abs(ey) < DEADBAND_Y:
                ey = 0.0

            # Convert error to angle delta
            # If your camera shows target to the RIGHT (ex>0), you typically need pan to increase or decrease
            # depending on your mechanics. Use PAN_DIR to invert quickly.
            d_pan = PAN_DIR * (ex * GAIN_PAN)
            d_tilt = TILT_DIR * (ey * GAIN_TILT)

            # Clamp per-loop step
            d_pan = clamp_step(d_pan, MAX_STEP_DEG)
            d_tilt = clamp_step(d_tilt, MAX_STEP_DEG)

            pan = clamp(pan + d_pan, PAN_MIN, PAN_MAX)
            tilt = clamp(tilt + d_tilt, TILT_MIN, TILT_MAX)

            sc.set(pan, tilt)

            # Optional: debug
            # print(f"cx={cx:.0f} cy={cy:.0f} ex={ex:+.2f} ey={ey:+.2f} -> pan={pan:.1f} tilt={tilt:.1f}")

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        pass
    finally:
        sc.close()
        print("\nClosed servos.")

if __name__ == "__main__":
    main()