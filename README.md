"""
Project Airbus 1 - Step C: X-Plane -> Arduino bridge for the PUSHROD design.

The model sits on a central ball pivot. Two servos push it up and down with
short rods: the pitch servo (pin 9) under the nose half, the roll servo
(pin 10) under the right half.

A servo horn swings in an arc, so the servo angle is NOT the same as the
platform tilt. This script works out the servo angle that gives the platform
tilt you want, using the horn length and where each rod joins the platform.

BEFORE RUNNING:
  1. Upload two_servos.ino (limits 45 to 135) and CLOSE the Serial Monitor.
  2. X-Plane: Settings > Data Output, row 17 ticked for UDP,
     Net Connections > Data: 127.0.0.1, port 49001.
  3. Close xplane_listener.py (it uses the same port).

RUN:  py xplane_to_arduino_pushrod.py
STOP: Ctrl+C  (the model returns to level)
"""

import math
import socket
import struct
import time

import serial  # from pyserial

# ---------------- Connection settings ----------------
XPLANE_PORT = 49001      # must match the port in X-Plane's Data settings
ARDUINO_PORT = "COM3"    # the port shown in the Arduino IDE
BAUD = 9600              # must match Serial.begin(9600) in the sketch

# ---------------- Safe servo limits (keep equal to the Arduino sketch) ----
CENTER = 90              # servo angle that means "horn level, platform level"
MIN_ANGLE = 45
MAX_ANGLE = 135

# ---------------- Pushrod geometry (cm) - measure your build ----------------
HORN_CM = 1.5            # shaft centre to the rod hole in the horn
PITCH_ARM_CM = 3.7       # pitch rod joins the platform this far AHEAD of the pivot
ROLL_ARM_CM = 4.5        # roll rod joins the platform this far to the RIGHT of the pivot

# ---------------- How much the platform tilts ----------------
MAX_TILT_DEG = 12.0      # the platform never tilts more than this
PITCH_SCALE = 1.0        # platform degrees per aircraft degree (1.0 = mirror it)
ROLL_SCALE = 0.5         # 0.5 = a 24 degree bank gives the full 12 degree platform tilt

# ---------------- Tuning ----------------
PITCH_DIR = -1           # -1 = reversed: nose up in the sim now raises the model nose (set to 1 to undo)
ROLL_DIR = 1             # change to -1 if it banks the wrong way
PITCH_FUDGE = 1.0        # multiply the servo movement (1.1 = 10 percent more)
ROLL_FUDGE = 1.0
PITCH_TRIM = 0           # servo degrees to make the platform sit level at rest
ROLL_TRIM = 0            # example: ROLL_TRIM = -6 if it rests banked

SEND_INTERVAL = 0.05     # seconds between messages (20 per second)
DATA_TIMEOUT = 1.0       # seconds without X-Plane data before going level
# ------------------------------------------------------------------------


def clamp(value, low, high):
    return max(low, min(high, value))


def servo_offset(tilt_deg, arm_cm):
    """Servo degrees (from centre) needed to tilt the platform by tilt_deg.

    The rod joint sits arm_cm from the pivot, so a tilt lifts it by
    arm * tan(tilt). The horn tip lifts by horn * sin(offset). Set equal.
    """
    rise = arm_cm * math.tan(math.radians(tilt_deg))
    ratio = clamp(rise / HORN_CM, -1.0, 1.0)
    return math.degrees(math.asin(ratio))


def to_servo(aircraft_angle, direction, scale, arm_cm, fudge, trim):
    """Aircraft angle (degrees) -> safe servo angle."""
    tilt = clamp(direction * scale * aircraft_angle, -MAX_TILT_DEG, MAX_TILT_DEG)
    angle = CENTER + trim + fudge * servo_offset(tilt, arm_cm)
    return int(clamp(round(angle), MIN_ANGLE, MAX_ANGLE))


def level(trim):
    return int(clamp(CENTER + trim, MIN_ANGLE, MAX_ANGLE))


def check_setup():
    """Print what full tilt needs, and warn if the servo limits would cut it off."""
    for name, arm, fudge, trim in (
        ("Pitch", PITCH_ARM_CM, PITCH_FUDGE, PITCH_TRIM),
        ("Roll", ROLL_ARM_CM, ROLL_FUDGE, ROLL_TRIM),
    ):
        reach = arm * math.tan(math.radians(MAX_TILT_DEG)) / HORN_CM
        off = fudge * servo_offset(MAX_TILT_DEG, arm)
        low, high = CENTER + trim - off, CENTER + trim + off
        print(f"{name}: full tilt (+/-{MAX_TILT_DEG:.0f} deg) needs servo {low:.0f} to {high:.0f}")
        if reach > 1.0:
            print(f"  WARNING: the horn is too short to reach that tilt. Lower MAX_TILT_DEG or lengthen the horn.")
        if low < MIN_ANGLE or high > MAX_ANGLE:
            print(f"  WARNING: outside {MIN_ANGLE} to {MAX_ANGLE}. Lower MAX_TILT_DEG, "
                  f"or widen the limits here AND in two_servos.ino.")


def main():
    # Listen for X-Plane
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", XPLANE_PORT))
    except OSError:
        print(f"Port {XPLANE_PORT} is already in use.")
        print("Close xplane_listener.py (and any other copy of this script), then try again.")
        return
    sock.settimeout(0.1)

    # Connect to the Arduino
    arduino = serial.Serial(ARDUINO_PORT, BAUD, timeout=1)
    time.sleep(2)  # the Arduino restarts when the port opens, so wait for it
    print(f"Connected to Arduino on {ARDUINO_PORT}. Waiting for X-Plane...")
    check_setup()

    pitch = 0.0
    roll = 0.0
    last_data = 0.0
    last_send = 0.0
    last_print = 0.0
    got_data = False

    try:
        while True:
            # 1. Read any data X-Plane has sent
            try:
                data, _ = sock.recvfrom(2048)
                if data[:4] == b"DATA":
                    for offset in range(5, len(data) - 35, 36):
                        index = struct.unpack_from("<i", data, offset)[0]
                        values = struct.unpack_from("<8f", data, offset + 4)
                        if index == 17:  # pitch, roll, headings
                            pitch = values[0]
                            roll = values[1]
                            last_data = time.time()
                            if not got_data:
                                print("X-Plane data received. Moving servos.")
                                got_data = True
            except socket.timeout:
                pass

            # 2. Send to the Arduino at a steady rate
            now = time.time()
            if now - last_send >= SEND_INTERVAL:
                if now - last_data > DATA_TIMEOUT:
                    # No data from X-Plane: go level (like a lost-datalink failsafe)
                    pitch_servo = level(PITCH_TRIM)
                    roll_servo = level(ROLL_TRIM)
                    if now - last_print > 3:
                        print("No data from X-Plane. Holding level. "
                              "Check X-Plane Data Output (row 17, UDP, 127.0.0.1, port 49001).")
                        last_print = now
                else:
                    pitch_servo = to_servo(pitch, PITCH_DIR, PITCH_SCALE,
                                           PITCH_ARM_CM, PITCH_FUDGE, PITCH_TRIM)
                    roll_servo = to_servo(roll, ROLL_DIR, ROLL_SCALE,
                                          ROLL_ARM_CM, ROLL_FUDGE, ROLL_TRIM)
                    if now - last_print > 1:
                        print(f"pitch {pitch:6.1f}  roll {roll:6.1f}  ->  servos {pitch_servo},{roll_servo}")
                        last_print = now

                arduino.write(f"{pitch_servo},{roll_servo}\n".encode())
                last_send = now

    except KeyboardInterrupt:
        print("\nStopping. Returning servos to level.")
        arduino.write(f"{level(PITCH_TRIM)},{level(ROLL_TRIM)}\n".encode())
        time.sleep(0.2)
    finally:
        arduino.close()
        sock.close()


if __name__ == "__main__":
    main()
