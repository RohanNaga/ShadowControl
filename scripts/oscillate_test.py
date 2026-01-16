#!/usr/bin/env python3
"""
Oscillate each servo one at a time to test direction and function.
Each servo moves +30°, then back to 0°.
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_control import create_controller, auto_detect_port, load_calibration, angle_to_steps

SERVO_NAMES = {
    5: "R_Shoulder",
    6: "R_Elbow",
    7: "R_Wrist",
    8: "L_Shoulder",
    9: "L_Elbow",
    10: "L_Wrist",
}

def main():
    port = auto_detect_port()
    if not port:
        print("No servo port found!")
        return

    print(f"Using port: {port}")

    ctrl = create_controller(port, 500000)
    ctrl.open()
    cal = load_calibration()

    print("\n" + "=" * 50)
    print("SERVO OSCILLATION TEST")
    print("Each servo: 0° → +30° → 0°")
    print("=" * 50 + "\n")

    # First, move all to zero
    print("Moving all servos to 0°...")
    for sid in SERVO_NAMES.keys():
        zero = cal[str(sid)]["zero_offset"]
        ctrl.write_position(sid, zero)
    time.sleep(1)

    # Test each servo one at a time
    for sid, name in SERVO_NAMES.items():
        zero = cal[str(sid)]["zero_offset"]

        print(f"\nTesting ID {sid} ({name}):")

        # Move to +30
        print(f"  → +30°")
        ctrl.write_position(sid, zero + angle_to_steps(30))
        time.sleep(0.7)

        # Back to 0
        print(f"  → 0°")
        ctrl.write_position(sid, zero)
        time.sleep(0.5)

    print("\n" + "=" * 50)
    print("TEST COMPLETE")
    print("=" * 50)

    ctrl.close()

if __name__ == "__main__":
    main()
