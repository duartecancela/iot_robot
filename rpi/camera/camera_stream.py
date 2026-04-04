import os
import time
import threading
from typing import Dict, Any, List, Optional, Tuple

import cv2
from flask import Flask, Response, jsonify
from picamera2 import Picamera2

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080

FRAME_W = 640
FRAME_H = 480
CONF_THRES = 0.45  # um pouco mais exigente para evitar falsos positivos

# ------------------------------------------------------------
# Target classes (COCO IDs for TF SSD label map)
# ------------------------------------------------------------
COCO_BOTTLE = 44
COCO_CELL_PHONE = 77

# Escolha do alvo:
#   TARGET=bottle
#   TARGET=cell_phone
#   TARGET=both
TARGET = os.getenv("TARGET", "bottle").strip().lower()

TARGET_CLASS_IDS = set()
if TARGET in ("bottle", "garrafa"):
    TARGET_CLASS_IDS = {COCO_BOTTLE}
elif TARGET in ("cell_phone", "phone", "telemovel", "telemóvel"):
    TARGET_CLASS_IDS = {COCO_CELL_PHONE}
else:
    TARGET_CLASS_IDS = {COCO_BOTTLE, COCO_CELL_PHONE}

ID_TO_LABEL = {
    COCO_BOTTLE: "bottle",
    COCO_CELL_PHONE: "cell_phone",
}

# ------------------------------------------------------------
# Orientation (FIXED)
# ------------------------------------------------------------
ROTATE_180 = True
HFLIP = False
VFLIP = False

# Se quiseres voltar a ter variáveis de ambiente para flip/rotate:
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
latest_detections: Dict[str, Any] = {"ts": None, "detections": []}


def apply_orientation(bgr):
    if HFLIP:
        bgr = cv2.flip(bgr, 1)
    if VFLIP:
        bgr = cv2.flip(bgr, 0)
    if ROTATE_180:
        bgr = cv2.rotate(bgr, cv2.ROTATE_180)
    return bgr


def load_net():
    if not os.path.exists(PB) or not os.path.exists(PBTXT):
        raise RuntimeError(
            f"Missing model files.\nExpected:\n  {PB}\n  {PBTXT}\n"
            "Download them into rpi/camera/models/."
        )

    net = cv2.dnn_DetectionModel(PB, PBTXT)
    net.setInputSize(320, 320)
    net.setInputScale(1.0 / 127.5)
    net.setInputMean((127.5, 127.5, 127.5))
    net.setInputSwapRB(True)
    return net


def _center_distance_sq(x: int, y: int, w: int, h: int) -> float:
    cx = x + w / 2.0
    cy = y + h / 2.0
    dx = cx - (FRAME_W / 2.0)
    dy = cy - (FRAME_H / 2.0)
    return dx * dx + dy * dy


def camera_loop():
    global latest_jpeg, latest_detections

    net = load_net()

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (FRAME_W, FRAME_H), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    t0 = time.time()
    frames = 0
    fps_est = 0.0

    while True:
        rgb = picam2.capture_array()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        bgr = apply_orientation(bgr)

        class_ids, confidences, boxes = net.detect(bgr, confThreshold=CONF_THRES, nmsThreshold=0.4)

        dets: List[Dict[str, Any]] = []
        if class_ids is not None and len(class_ids) > 0:
            for cid, conf, box in zip(class_ids.flatten(), confidences.flatten(), boxes):
                cid = int(cid)
                if cid not in TARGET_CLASS_IDS:
                    continue

                x, y, w, h = [int(v) for v in box]
                label = ID_TO_LABEL.get(cid, str(cid))

                dets.append(
                    {
                        "label": label,
                        "x": x,
                        "y": y,
                        "w": w,
                        "h": h,
                        "conf": round(float(conf), 3),
                    }
                )

        # (Opcional) ordenar por "mais perto do centro" para o tracker apanhar o alvo certo
        dets.sort(key=lambda d: _center_distance_sq(d["x"], d["y"], d["w"], d["h"]))

        # desenhar boxes
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
                0.6,
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
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with lock:
                latest_jpeg = jpg.tobytes()
                latest_detections = {"ts": time.time(), "detections": dets}


def mjpeg_generator():
    while True:
        with lock:
            frame = latest_jpeg
        if frame is None:
            time.sleep(0.05)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Cache-Control: no-cache\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/stream.mjpg")
def stream():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/detections")
def detections():
    with lock:
        return jsonify(dict(latest_detections))


@app.route("/health")
def health():
    return jsonify({"ok": True})


if __name__ == "__main__":
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)