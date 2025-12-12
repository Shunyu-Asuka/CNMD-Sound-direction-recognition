import time
import math
import struct
import cv2
import usb.core
import usb.util
import lgpio


# ================================
# XVF3800 DOA PARAMETERS
# ================================
PARAMETERS = {
    "AEC_AZIMUTH_VALUES": (33, 75, 16, "ro", "radians"),
}

class ReSpeaker:
    TIMEOUT = 100000

    def __init__(self, dev):
        self.dev = dev

    def read(self, name):
        resid, cmdid_raw, length, _, dtype = PARAMETERS[name]
        cmdid = 0x80 | cmdid_raw
        length += 1  # add status byte

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmdid, resid, length, self.TIMEOUT
        )

        data = response.tobytes()

        if dtype == "radians":
            fmt = "<" + "f" * int((length - 1) / 4)
            return struct.unpack(fmt, data[1:length])
        return data[1:length]


def find_respeaker():
    dev = usb.core.find(idVendor=0x2886)
    return ReSpeaker(dev) if dev else None


# ================================
# STEPPER MOTOR SETTINGS
# ================================
# Pins (BCM numbering)
IN1 = 23
IN2 = 24
IN3 = 25
IN4 = 16
PINS = [IN1, IN2, IN3, IN4]

# Half-step sequence (8-step)
SEQ = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1],
]

CHIP = 0


STEPS_PER_REV = 800

STEP_DELAY = 0.0015
MIN_MOVE_DEG = 5.0
EXTRA_SETTLE = 0.30

# Camera mounting offset relative to mic DOA reference
CAMERA_OFFSET_DEG = 0.0

current_angle_deg = 0.0
busy_until = 0.0


# ================================
# HELPER FUNCTIONS
# ================================
def normalize(angle):
    """Normalize angle to 0–360 degrees."""
    return (angle + 360) % 360

def angle_to_steps(delta_deg):
    """Convert degrees to stepper motor steps."""
    return int(delta_deg * (STEPS_PER_REV / 360.0))


# ================================
# STEPPER MOTOR CONTROL
# ================================
def stepper_setup():
    """Initialize GPIO for the stepper motor."""
    global h
    h = lgpio.gpiochip_open(CHIP)
    for p in PINS:
        lgpio.gpio_claim_output(h, p, 0)

def stepper_cleanup():
    """Release GPIO pins."""
    for p in PINS:
        try:
            lgpio.gpio_write(h, p, 0)
            lgpio.gpio_free(h, p)
        except:
            pass
    lgpio.gpiochip_close(h)

def move_steps(steps):
    """Rotate motor by given steps. Positive = CW, Negative = CCW."""
    if steps == 0:
        return

    direction = 1 if steps > 0 else -1
    seq = SEQ[::direction]

    for _ in range(abs(steps)):
        for patt in seq:
            for pin, val in zip(PINS, patt):
                lgpio.gpio_write(h, pin, val)
            time.sleep(STEP_DELAY)


# ================================
# MAIN LOOP — DOA + STEPPER + CAMERA
# ================================
def main():
    global current_angle_deg, busy_until

    # Initialize stepper
    try:
        stepper_setup()
    except Exception as e:
        print("Failed to initialize GPIO:", e)
        return

    # Initialize XVF3800 DOA mic
    dev = find_respeaker()
    if not dev:
        print("XVF3800 Microphone not found!")
        return

    # Initialize camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Failed to open camera!")
        return

    print("\n=== DOA + Stepper Motor + Camera System Started ===")
    print("Press Q to exit.\n")

    try:
        while True:

            # ===== CAMERA FRAME (non-blocking) =====
            ret, frame = cap.read()
            if ret:
                cv2.imshow("Camera View", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            now = time.time()

            # Stepper is busy, wait before next DOA reading
            if now < busy_until:
                continue

            # ===== READ DOA =====
            vals = dev.read("AEC_AZIMUTH_VALUES")
            doa_rad = vals[1]
            doa_deg = normalize(doa_rad * 180 / math.pi)

            target_deg = normalize(doa_deg + CAMERA_OFFSET_DEG)

            # ===== NEW LOGIC: DO NOT COMPUTE SHORTEST PATH =====
            delta_deg = target_deg - current_angle_deg

            print(f"DOA={doa_deg:.1f}° Current={current_angle_deg:.1f}° Target={target_deg:.1f}° Δ={delta_deg:.1f}°")

            # Ignore small movement
            if abs(delta_deg) < MIN_MOVE_DEG:
                continue

            steps = angle_to_steps(delta_deg)
            move_steps(steps)

            # Update angle (keep it 0-360)
            current_angle_deg = normalize(current_angle_deg + delta_deg)

            # Motor settle time
            busy_until = time.time() + EXTRA_SETTLE

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        stepper_cleanup()
        print("System exited. GPIO released.")


if __name__ == "__main__":
    main()
