import time
from servo_controller import ServoController

s = ServoController()

try:
    while True:
        s.set(0, 0);     time.sleep(1)
        s.set(90, 90);   time.sleep(1)
        s.set(180, 180); time.sleep(1)
except KeyboardInterrupt:
    s.center()
    s.close()
    print("Stopped.")
