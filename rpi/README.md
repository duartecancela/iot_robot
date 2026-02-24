# Raspberry Pi Layer

This folder contains the **Raspberry Pi hardware + vision tracking layer**
of the `iot_robot` project.

The Raspberry Pi is responsible for:

- Pan/Tilt servo control (PCA9685)
- Camera vision processing (OpenCV DNN - COCO)
- Object tracking (center error → servo control)
- MJPEG streaming + JSON metadata
- Future: laser activation + MQTT publishing

---

# Architecture Overview

Camera (Picamera2)
        ↓
OpenCV DNN (SSD MobileNet v3 COCO)
        ↓
Filter target class (bottle / cell_phone)
        ↓
Bounding box → center (cx, cy)
        ↓
Tracking loop (EMA + deadband + hysteresis)
        ↓
Servo controller (PCA9685)

Only ONE process must access the camera at a time.

---

# Folder Structure

rpi/
├── servos/
│   ├── servo_controller.py
│   ├── test_servos.py
│   └── tracker_step3_servo.py
├── camera/
│   ├── opencv_dnn_mjpeg_person.py
│   └── models/
│       ├── frozen_inference_graph.pb
│       └── ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt
├── venv/
├── requirements.txt
└── README.md

---

# 1) Enable I2C (PCA9685)

sudo raspi-config  
→ Interface Options  
→ I2C → Enable  
sudo reboot  

Verify:

sudo apt install i2c-tools  
i2cdetect -y 1  

Expected address:
0x40

---

# 2) Install System Dependencies

sudo apt update  
sudo apt install -y \
    python3-picamera2 \
    python3-opencv \
    python3-numpy \
    python3-simplejpeg \
    ffmpeg

---

# 3) Python Virtual Environment

Inside `rpi/`:

python3 -m venv venv --system-site-packages  
source venv/bin/activate  
pip install --upgrade pip  

Install required packages:

pip install \
    adafruit-blinka \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    flask \
    requests  

pip freeze > requirements.txt  

NOTE: No torch / ultralytics installed on Raspberry Pi.

---

# 4) Download Detection Model

Models are NOT included in repo.

mkdir -p camera/models  
cd camera/models  

curl -L -o frozen_inference_graph.pb \
https://raw.githubusercontent.com/ankityddv/ObjectDetector-OpenCV/main/frozen_inference_graph.pb  

curl -L -o ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt \
https://raw.githubusercontent.com/ankityddv/ObjectDetector-OpenCV/main/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt  

---

# 5) Vision Server

Run:

source venv/bin/activate  
TARGET=cell_phone python camera/opencv_dnn_mjpeg_person.py  

Supported targets:

TARGET=bottle  
TARGET=cell_phone  
TARGET=both  

Stream:
http://<PI_IP>:8080/stream.mjpg  

Detections:
http://<PI_IP>:8080/detections  

---

# 6) Tracking (Servo Follow)

In another terminal:

source venv/bin/activate  
python servos/tracker_step3_servo.py  

Tracking features:

- Lock-on to last target
- EMA smoothing
- Hysteresis deadband
- Per-axis max step clamp
- Configurable gains

---

# 7) Stability Tuning

If oscillation occurs:

Adjust in tracker_step3_servo.py:

EMA_ALPHA          # lower = smoother
MAX_STEP_PAN       # reduce if hunting
MAX_STEP_TILT      # reduce if vertical jitter
GAIN_PAN           # lower = less aggressive
GAIN_TILT          # lower = less aggressive

Typical stable values:

EMA_ALPHA = 0.15–0.25  
MAX_STEP_PAN = 1.2–1.6  
MAX_STEP_TILT = 1.0–1.3  

---

# 8) Hardware Wiring

PCA9685:

- VCC → 3.3V (RPi)
- GND → GND (shared with servo PSU)
- SDA → GPIO2
- SCL → GPIO3

Servos:

- Powered from external 5V
- DO NOT power from Raspberry Pi 5V rail
- Common GND mandatory

---

# 9) Production Checklist

Before installing on robot:

- Calibrate PAN_CENTER and TILT_CENTER
- Verify mechanical limits
- Secure cables (strain relief)
- Ensure common ground stability
- Keep servo power wires away from camera ribbon

---

# Next Step

Backend + Frontend deployment on Raspberry Pi:

- Install Node.js
- Install backend dependencies
- Build frontend
- Serve frontend via backend or Nginx
- Optional: use PM2 for production