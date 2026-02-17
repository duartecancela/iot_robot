import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoController:
    def __init__(self, pan_ch=0, tilt_ch=1, freq=50):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = freq
        self.pan = servo.Servo(self.pca.channels[pan_ch])
        self.tilt = servo.Servo(self.pca.channels[tilt_ch])

    def set(self, pan_angle: float, tilt_angle: float):
        self.pan.angle = pan_angle
        self.tilt.angle = tilt_angle

    def center(self):
        self.set(90, 90)

    def close(self):
        self.pca.deinit()
