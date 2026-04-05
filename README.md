# Raspberry Pi Layer

This folder contains the **Raspberry Pi hardware + vision tracking layer**
of the `iot_robot` project.

The Raspberry Pi is responsible for:

- Pan/Tilt servo control (PCA9685)
- Camera vision processing (OpenCV DNN - COCO)
- Object tracking (center error → servo control)
- MJPEG streaming + JSON metadata
- Laser control via GPIO + MOSFET
- Laser firing logic based on target stabilization
- Future: MQTT publishing + full system integration

---

# Current Status

✔ Servos (Pan/Tilt) working  
✔ PCA9685 communication working (I2C)  
✔ GPIO control validated  
✔ Laser control via MOSFET working  
✔ Camera streaming (MJPEG) working  
✔ Object detection (SSD MobileNet v3 COCO) working  
✔ JSON detections endpoint working  
✔ Tracking (Pan + Tilt) functional and stabilized  
✔ Laser controller implemented (3x blink)  
✔ Laser firing integrated with tracking  
✔ Project structure organized  

---

# Architecture Overview

Camera (Picamera2)  
↓  
OpenCV DNN (SSD MobileNet v3 COCO)  
↓  
Filter target class (cell_phone)  
↓  
Bounding box → center (cx, cy)  
↓  
Tracking loop (EMA + deadband + hysteresis)  
↓  
Servo controller (PCA9685)  
↓  
Laser controller (GPIO18 → MOSFET)  
↓  
Fire logic (centered + stable + cooldown)

Only ONE process must access the camera at a time.

---

# Folder Structure

```
rpi/
├── camera/
│   ├── camera_stream.py
│   └── models/
├── laser/
│   └── laser_controller.py
├── servos/
│   └── servo_controller.py
├── tracking/
│   └── tracking_controller.py
├── tests/
│   ├── test_servo_pan_manual.py
│   ├── test_servo_tilt_manual.py
│   └── test_laser_gpio.py
├── venv/
├── requirements.txt
└── README.md
```

---

# 1) Enable I2C (PCA9685)

```
sudo raspi-config
→ Interface Options
→ I2C → Enable
sudo reboot
```

Verify:

```
sudo apt install i2c-tools
i2cdetect -y 1
```

Expected address:
```
0x40
```

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

Controls:

- PAN → A / D  
- TILT → W / S  

---

# 4) Laser Testing (GPIO 18)

Basic GPIO test:

```
python3 tests/test_laser_gpio.py
```

Blink test (3 pulses):

```
python -m laser.laser_controller
```

Expected behavior:

- Laser blinks 3 times
- Returns to OFF state

---

# 5) Vision Server

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

Notes:

- Resolution optimized for low latency (320x240)
- Reduced JPEG quality for performance
- Detection tuned for stability vs responsiveness

---

# 6) Tracking (Servo Follow)

Run in another terminal:

```
python -m tracking.tracking_controller
```

Tracking features:

- Target lock-on (based on previous position)
- EMA smoothing (reduces jitter)
- Hysteresis deadband (prevents oscillation)
- Limited step movement (stability control)
- Separate tuning for pan and tilt
- Target loss tolerance (short-term memory)
- Confidence filtering

---

# 7) Laser Firing Logic

Laser is triggered only when:

- A valid target is detected
- Target is near the center of the frame
- Target remains stable for a short period (hold time)
- System is not in cooldown

Behavior:

- Laser blinks 3 times
- Non-blocking execution (threaded)
- Cooldown prevents repeated firing
- Requires target reacquisition before next firing

---

# 8) Hardware Wiring

## PCA9685

- VCC → 3.3V (RPi)
- GND → GND (shared)
- SDA → GPIO2 (Pin 3)
- SCL → GPIO3 (Pin 5)

## Servos

- External 5V supply required
- DO NOT power from Raspberry Pi
- Common GND mandatory

## Laser (MOSFET)

- GPIO18 (Pin 12) → Gate
- GND → Common GND
- Laser powered externally (5V)

Recommended:

- 220Ω resistor (GPIO → Gate)
- 10kΩ pull-down (Gate → GND)

---

# Important Notes

- GPIO numbering uses BCM mode
- GPIO18 = physical Pin 12
- Do NOT confuse GPIO number with pin number
- Only one process can use the camera at a time
- Tracking behavior depends heavily on tuning parameters
- MJPEG stream may introduce slight latency (browser-dependent)

---

# Next Steps

- Implement state machine (SCAN / TRACK / FIRE)
- Improve detection robustness (reduce false positives)
- Add MQTT integration
- Connect to backend + frontend
- Extend detection to birds (outdoor scenario)
- Improve tracking responsiveness and accuracy