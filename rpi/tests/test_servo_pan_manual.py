from servos.servo_controller import ServoController
import time

sc = ServoController()

pan = 90.0          # começa em 90
tilt = 120.0        # o teu centro calibrado

print("Use A/D to move PAN")
print("Q to quit")

try:
    while True:
        sc.set(pan, tilt)
        print(f"PAN: {pan:.1f}", end="\r")

        cmd = input()

        if cmd.lower() == "a":
            pan -= 1
        elif cmd.lower() == "d":
            pan += 1
        elif cmd.lower() == "q":
            break

except KeyboardInterrupt:
    pass
finally:
    sc.close()