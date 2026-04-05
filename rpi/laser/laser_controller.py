import RPi.GPIO as GPIO
import time

LASER_PIN = 18

def blink_laser(times=3, on_time=0.2, off_time=0.2):
    for _ in range(times):
        GPIO.output(LASER_PIN, GPIO.HIGH)
        time.sleep(on_time)
        GPIO.output(LASER_PIN, GPIO.LOW)
        time.sleep(off_time)

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LASER_PIN, GPIO.OUT, initial=GPIO.LOW)

    try:
        print("Blinking laser 3 times...")
        blink_laser(times=3, on_time=0.2, off_time=0.2)
        print("Done.")
    finally:
        GPIO.output(LASER_PIN, GPIO.LOW)
        GPIO.cleanup()

if __name__ == "__main__":
    main()