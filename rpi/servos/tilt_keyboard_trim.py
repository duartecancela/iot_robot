from servo_controller import ServoController

PAN = 90
tilt = 120

STEP = 2  # 2 graus por toque

def main():
    global tilt
    sc = ServoController()
    sc.set(PAN, tilt)

    print("Keys: w=tilt up (decrease), s=tilt down (increase), q=quit")
    print(f"Start tilt={tilt}")

    try:
        while True:
            k = input("> ").strip().lower()
            if k == "w":
                tilt -= STEP
            elif k == "s":
                tilt += STEP
            elif k == "q":
                break
            else:
                print("Use w/s/q")
                continue

            tilt = max(0, min(180, tilt))
            sc.set(PAN, tilt)
            print(f"tilt={tilt}")

    finally:
        sc.set(PAN, tilt)
        sc.close()
        print(f"Final tilt={tilt}")

if __name__ == "__main__":
    main()