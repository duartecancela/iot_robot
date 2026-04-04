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

```
rpi/
├── camera/
│   ├── camera_stream.py
│   └── models/
├── servos/
│   └── servo_controller.py
├── tracking/
│   └── tracking_controller.py
├── tests/
│   ├── test_servo_pan_manual.py
│   └── test_servo_tilt_manual.py
├── venv/
├── requirements.txt
└── README.md
```

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

# 2) Python Virtual Environment

Inside `rpi/`:

```
python3 -m venv venv --system-site-packages  
source venv/bin/activate  
pip install --upgrade pip  
```

Install required packages:

```
pip install \
    adafruit-blinka \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    flask \
    requests
```

---

# 3) Servo Testing

Run manual tests:

```
python -m tests.test_servo_pan_manual
python -m tests.test_servo_tilt_manual
```

---

# 4) Vision Server

Run:

```
TARGET=cell_phone python -m camera.camera_stream
```

Stream:
```
http://<PI_IP>:8080/stream.mjpg
```

Detections:
```
http://<PI_IP>:8080/detections
```

---

# 5) Tracking (Servo Follow)

Run in another terminal:

```
python -m tracking.tracking_controller
```

Tracking features:

- Lock-on to last target
- EMA smoothing
- Hysteresis deadband
- Per-axis max step clamp
- Configurable gains

---

# 6) Stability Tuning

Adjust in `tracking_controller.py`:

- EMA_ALPHA
- MAX_STEP_PAN
- MAX_STEP_TILT
- GAIN_PAN
- GAIN_TILT

---

# 7) Hardware Wiring

PCA9685:

- VCC → 3.3V (RPi)
- GND → GND (shared with servo PSU)
- SDA → GPIO2
- SCL → GPIO3

Servos:

- Powered from external 5V
- DO NOT power from Raspberry Pi
- Common GND mandatory

---

# Next Steps

- Laser integration
- State machine (AUTO_SCAN / TRACK / FIRE)
- MQTT communication
- Frontend integration