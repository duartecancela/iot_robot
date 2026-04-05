import os
import time
import threading
from typing import Dict, Any, List

import cv2
from flask import Flask, Response, jsonify
from picamera2 import Picamera2

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

FRAME_W = 320
FRAME_H = 240
CONF_THRES = 0.35  # Slightly lower threshold to recover detections
JPEG_QUALITY = 60

# ------------------------------------------------------------
# Target classes (COCO IDs for TF SSD label map)
# ------------------------------------------------------------
COCO_BOTTLE = 44
COCO_BIRD = 16
COCO_CELL_PHONE = 77

DEFAULT_TARGET = "cell_phone"

# Environment variable examples:
#   TARGET=cell_phone
#   TARGET=bird
#   TARGET=bottle
#   TARGET=all
TARGET = os.getenv("TARGET", DEFAULT_TARGET).strip().lower()

TARGET_MAP = {
    "bottle": {COCO_BOTTLE},
    "bird": {COCO_BIRD},
    "cell_phone": {COCO_CELL_PHONE},
    "phone": {COCO_CELL_PHONE},
    "all": {COCO_BOTTLE, COCO_BIRD, COCO_CELL_PHONE},
}

TARGET_CLASS_IDS = TARGET_MAP.get(TARGET, {COCO_CELL_PHONE})

ID_TO_LABEL = {
    COCO_BOTTLE: "bottle",
    COCO_BIRD: "bird",
    COCO_CELL_PHONE: "cell_phone",
}

# ------------------------------------------------------------
# Image orientation configuration
# ------------------------------------------------------------
ROTATE_180 = True
HFLIP = False
VFLIP = False

# Optional environment override for orientation settings
ALLOW_ENV_OVERRIDE = False
if ALLOW_ENV_OVERRIDE:
    ROTATE_180 = int(os.getenv("VISION_ROTATE_180", "1")) == 1
    HFLIP = int(os.getenv("VISION_HFLIP", "0")) == 1
    VFLIP = int(os.getenv("VISION_VFLIP", "0")) == 1

MODEL_DIR = os.path.join(os.path.dirname(__file__), "models")
PB = os.path.join(MODEL_DIR, "frozen_inference_graph.pb")
PBTXT = os.path.join(MODEL_DIR, "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt")

app = Flask(__name__)

lock = threading.Lock()
latest_jpeg: bytes | None = None
latest_detections: Dict[str, Any] = {"ts": None, "detections": [], "primary_target": None}


def apply_orientation(bgr):
    """Apply configured orientation transformations to the frame."""
    if HFLIP:
        bgr = cv2.flip(bgr, 1)
    if VFLIP:
        bgr = cv2.flip(bgr, 0)
    if ROTATE_180:
        bgr = cv2.rotate(bgr, cv2.ROTATE_180)
    return bgr


def load_net():
    """Load the DNN model for object detection."""
    if not os.path.exists(PB) or not os.path.exists(PBTXT):
        raise RuntimeError(
            f"Missing model files.\nExpected:\n  {PB}\n  {PBTXT}\n"
            "Ensure the model files are available in rpi/camera/models/."
        )

    net = cv2.dnn_DetectionModel(PB, PBTXT)
    net.setInputSize(320, 320)
    net.setInputScale(1.0 / 127.5)
    net.setInputMean((127.5, 127.5, 127.5))
    net.setInputSwapRB(True)
    return net


def _center_distance_sq(x: int, y: int, w: int, h: int) -> float:
    """Compute squared distance from bounding box center to frame center."""
    cx = x + w / 2.0
    cy = y + h / 2.0
    dx = cx - (FRAME_W / 2.0)
    dy = cy - (FRAME_H / 2.0)
    return dx * dx + dy * dy


def run_detection(net, bgr) -> List[Dict[str, Any]]:
    """Run object detection and return filtered detections."""
    class_ids, confidences, boxes = net.detect(
        bgr,
        confThreshold=CONF_THRES,
        nmsThreshold=0.4,
    )

    dets: List[Dict[str, Any]] = []
    if class_ids is not None and len(class_ids) > 0:
        for cid, conf, box in zip(class_ids.flatten(), confidences.flatten(), boxes):
            cid = int(cid)
            if cid not in TARGET_CLASS_IDS:
                continue

            x, y, w, h = [int(v) for v in box]
            label = ID_TO_LABEL.get(cid, str(cid))
            cx = x + w // 2
            cy = y + h // 2

            dets.append(
                {
                    "label": label,
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "cx": cx,
                    "cy": cy,
                    "conf": round(float(conf), 3),
                }
            )

    dets.sort(key=lambda d: _center_distance_sq(d["x"], d["y"], d["w"], d["h"]))
    return dets


def camera_loop():
    """Main loop for frame capture, detection, and streaming."""
    global latest_jpeg, latest_detections

    net = load_net()

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"},
        buffer_count=1,
    )
    picam2.configure(config)
    picam2.start()

    try:
        picam2.set_controls({"FrameDurationLimits": (33333, 33333)})
    except Exception:
        pass

    t0 = time.time()
    frames = 0
    fps_est = 0.0

    while True:
        rgb = picam2.capture_array()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        bgr = apply_orientation(bgr)

        dets = run_detection(net, bgr)

        for d in dets:
            x, y, w, h = d["x"], d["y"], d["w"], d["h"]
            label = d["label"]
            conf = d["conf"]

            cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                bgr,
                f"{label} {conf:.2f}",
                (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        frames += 1
        t1 = time.time()
        if (t1 - t0) >= 1.0:
            fps_est = frames / (t1 - t0)
            frames = 0
            t0 = t1

        cv2.putText(
            bgr,
            f"FPS: {fps_est:.1f} | TARGET={TARGET}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ok, jpg = cv2.imencode(
            ".jpg",
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )

        if ok:
            with lock:
                latest_jpeg = jpg.tobytes()
                latest_detections = {
                    "ts": time.time(),
                    "detections": dets,
                    "primary_target": dets[0] if dets else None,
                }


def mjpeg_generator():
    """Generate MJPEG stream from latest frames."""
    while True:
        with lock:
            frame = latest_jpeg

        if frame is None:
            time.sleep(0.02)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Cache-Control: no-cache, no-store, must-revalidate\r\n"
            b"Pragma: no-cache\r\n"
            b"Expires: 0\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/stream.mjpg")
def stream():
    """HTTP endpoint for MJPEG video stream."""
    response = Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response


@app.route("/detections")
def detections():
    """HTTP endpoint returning latest detections in JSON."""
    with lock:
        return jsonify(dict(latest_detections))


@app.route("/health")
def health():
    """Health check endpoint."""
    with lock:
        has_frame = latest_jpeg is not None
    return jsonify({"ok": True, "has_frame": has_frame, "target": TARGET})


if __name__ == "__main__":
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)