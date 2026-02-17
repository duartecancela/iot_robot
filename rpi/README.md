# Raspberry Pi Layer

This folder contains the **Raspberry Pi hardware control layer** of the `iot_robot` project.

The Raspberry Pi is responsible for:

- Servo control (pan/tilt gimbal)
- Camera vision processing (future)
- Laser control (future)
- High-level coordination logic

---

## Folder Structure

```
rpi/
├── servos/
│   ├── servo_controller.py
│   └── test_servos.py
├── venv/              # Python virtual environment (ignored)
├── requirements.txt
└── README.md
```

---

## Initial Setup (Raspberry Pi)

### 1) Enable I2C

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

### 2) Create Python Virtual Environment

From inside `rpi/`:

```
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
```

Install dependencies:

```
pip install adafruit-blinka \
            adafruit-circuitpython-pca9685 \
            adafruit-circuitpython-motor
pip freeze > requirements.txt
```

---

## Servo Test

Run:

```
source venv/bin/activate
python servos/test_servos.py
```

This will move both servos:
0° → 90° → 180° (loop)

Stop with:
```
CTRL + C
```

---

## Hardware Notes

- PCA9685 VCC → 3.3V (RPi)
- PCA9685 SDA → GPIO2
- PCA9685 SCL → GPIO3
- Servos powered from **external 5V**
- Common GND between PSU and Raspberry Pi

Never power servos directly from Raspberry Pi 5V rail.

---

## Future Extensions

- Camera tracking integration
- MQTT integration
- Laser control module
- systemd service for auto-start

---
