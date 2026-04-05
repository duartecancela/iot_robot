import time
import RPi.GPIO as GPIO

LASER_PIN = 18

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(LASER_PIN, GPIO.OUT, initial=GPIO.LOW)

def blink_laser(times=3, on_time=0.2, off_time=0.2):
    for _ in range(times):
        GPIO.output(LASER_PIN, GPIO.HIGH)
        time.sleep(on_time)
        GPIO.output(LASER_PIN, GPIO.LOW)
        time.sleep(off_time)

def fire():
    blink_laser(times=3, on_time=0.2, off_time=0.2)

def laser_off():
    GPIO.output(LASER_PIN, GPIO.LOW)

if __name__ == "__main__":
    try:
        fire()
    finally:
        laser_off()