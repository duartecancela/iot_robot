import RPi.GPIO as GPIO
import time

PIN = 18

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)

print("LOW for 10 seconds")
GPIO.output(PIN, GPIO.LOW)
time.sleep(10)

print("HIGH for 10 seconds")
GPIO.output(PIN, GPIO.HIGH)
time.sleep(10)

print("LOW for 10 seconds")
GPIO.output(PIN, GPIO.LOW)
time.sleep(10)

GPIO.cleanup()