# Raspberry Pi Layer

This folder contains the **Raspberry Pi hardware and vision control layer**
of the `iot_robot` project.

The Raspberry Pi is responsible for:

- Servo control (pan/tilt gimbal)
- Camera vision processing (OpenCV DNN object detection)
- Laser control (future)
- High-level coordination logic

---

## Folder Structure

```
rpi/
├── servos/
│   ├── servo_controller.py
│   └── test_servos.py
├── camera/
│   ├── opencv_dnn_mjpeg_person.py
│   ├── yolo_mjpeg_person.py   # deprecated (torch not used on Pi)
│   └── models/
│       ├── frozen_inference_graph.pb
│       └── ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt
├── venv/
├── requirements.txt
└── README.md
```

---

## Initial Setup (Raspberry Pi)

### 1) Enable I2C (for PCA9685)

```
sudo raspi-config
→ Interface Options
→ I2C
→ Enable
sudo reboot
```

Verify device:

```
sudo apt install i2c-tools
i2cdetect -y 1
```

Expected address:
```
0x40
```

---

## Camera Dependencies (System Level)

Install camera and OpenCV at system level:

```
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-simplejpeg ffmpeg
```

---

## Create Python Virtual Environment

We use `--system-site-packages` so the venv can access `picamera2`
and system OpenCV/Numpy.

From inside `rpi/`:

```
python3 -m venv venv --system-site-packages
source venv/bin/activate
pip install --upgrade pip
```

Install dependencies:

```
pip install adafruit-blinka \
            adafruit-circuitpython-pca9685 \
            adafruit-circuitpython-motor \
            flask
pip freeze > requirements.txt
```

⚠️ We intentionally do NOT install torch or ultralytics on Raspberry Pi.

---

## Object Detection (OpenCV DNN - COCO)

We use **SSD MobileNet v3 COCO** via OpenCV DNN.

Download model files into:

```
rpi/camera/models/
```

Required files:

- `frozen_inference_graph.pb` (~20MB)
- `ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt`

---

## Vision Test (Person Detection + MJPEG Streaming)

Run:

```
source venv/bin/activate
python camera/opencv_dnn_mjpeg_person.py
```

Open in browser or VLC:

```
http://<PI_IP>:8080/stream.mjpg
```

Detection metadata:

```
http://<PI_IP>:8080/detections
```

This pipeline:

Camera → OpenCV DNN → Filter classes → Draw boxes → MJPEG Stream → JSON Metadata

Currently detecting:

- `person`
- `bird` (supported, future activation)

---

## Camera Orientation

If image is inverted:

```
VISION_ROTATE_180=1 python camera/opencv_dnn_mjpeg_person.py
```

Optional flips:

```
VISION_HFLIP=1
VISION_VFLIP=1
```

---

## Servo Test

```
source venv/bin/activate
python servos/test_servos.py
```

---

## Hardware Notes

- PCA9685 VCC → 3.3V (RPi)
- PCA9685 SDA → GPIO2
- PCA9685 SCL → GPIO3
- Servos powered from external 5V
- Common GND between PSU and Raspberry Pi

Never power servos directly from Raspberry Pi 5V rail.

---

### Download Model Files

Models are NOT included in the repository.

Download them manually into:

rpi/camera/models/

Commands:

```
mkdir -p rpi/camera/models
cd rpi/camera/models

curl -L -o frozen_inference_graph.pb \
https://raw.githubusercontent.com/ankityddv/ObjectDetector-OpenCV/main/frozen_inference_graph.pb

curl -L -o ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt \
https://raw.githubusercontent.com/ankityddv/ObjectDetector-OpenCV/main/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt
```


## Architecture Notes

Only one process should access the camera at a time.

Current vision architecture:

Camera → OpenCV DNN → Bounding Boxes → JSON → MJPEG Stream

Next step:
- Compute target center (cx, cy)
- Apply smoothing / deadband
- Drive servos (pan/tilt tracking)

Future upgrades:
- Replace MJPEG with WebRTC
- Replace SSD with YOLO ONNX (no torch)
- MQTT metadata publishing
