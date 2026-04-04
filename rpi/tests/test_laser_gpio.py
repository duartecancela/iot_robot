import RPi.GPIO as GPIO
import time

PIN = 18

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)

print("Blinking laser (CTRL+C to stop)")

try:
    while True:
        GPIO.output(PIN, GPIO.HIGH)
        print("ON")
        time.sleep(1)

        GPIO.output(PIN, GPIO.LOW)
        print("OFF")
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    GPIO.output(PIN, GPIO.LOW)
    GPIO.cleanup()