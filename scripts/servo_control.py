#!/usr/bin/env python3
"""
Servo calibration and angle control.

Usage:
    # Calibrate: save current positions as zero
    python servo_control.py --calibrate

    # Move servo to angle (relative to calibrated zero)
    python servo_control.py --id 5 --angle 45
    python servo_control.py --id 5,6,7 --angle 0    # Return to zero

    # Move all calibrated servos to angle
    python servo_control.py --all --angle 0

    # Show current angles
    python servo_control.py --status
"""

import argparse
import json
import os
import sys
import time

# Add parent directory for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.servo_controller import (
    ServoController,
    auto_detect_port,
    angle_to_steps,
    steps_to_angle,
    STEPS_PER_REV,
    DEFAULT_BAUDRATE as BAUDRATE,
)

# Config file for zero offsets
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "servo_calibration.json")


def load_calibration():
    """Load calibration data from file."""
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r") as f:
            return json.load(f)
    return {}


def save_calibration(cal):
    """Save calibration data to file."""
    with open(CONFIG_FILE, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"Saved to {CONFIG_FILE}")


def calibrate(ctrl, servo_ids):
    """Calibrate servos: save current position as zero."""
    print("=" * 50)
    print("CALIBRATING SERVOS")
    print("Current positions will be saved as zero (0°)")
    print("=" * 50)

    cal = load_calibration()

    for sid in servo_ids:
        success, _ = ctrl.ping(sid)
        if not success:
            print(f"  ID {sid}: NOT RESPONDING")
            continue

        pos = ctrl.read_position_raw(sid)
        if pos is None:
            print(f"  ID {sid}: FAILED TO READ")
            continue

        cal[str(sid)] = {
            "zero_offset": pos,
            "name": cal.get(str(sid), {}).get("name", f"servo_{sid}")
        }
        print(f"  ID {sid}: zero = {pos} (raw steps)")

    save_calibration(cal)
    print("\nCalibration complete!")


def move_to_angle(ctrl, servo_ids, angle):
    """Move servos to specified angle (relative to calibrated zero)."""
    cal = load_calibration()

    print(f"Moving to {angle}°...")
    for sid in servo_ids:
        key = str(sid)
        if key not in cal:
            print(f"  ID {sid}: NOT CALIBRATED (run --calibrate first)")
            continue

        zero_offset = cal[key]["zero_offset"]
        target_steps = zero_offset + angle_to_steps(angle)

        ctrl.write_position_raw(sid, target_steps)
        print(f"  ID {sid}: {angle}° → raw {target_steps}")

    time.sleep(0.5)
    print("Done.")


def show_status(ctrl, servo_ids):
    """Show current angle of each servo."""
    cal = load_calibration()

    print("=" * 50)
    print("SERVO STATUS")
    print("=" * 50)
    print(f"{'ID':<5} {'Name':<15} {'Raw Pos':<10} {'Angle':<10} {'Calibrated'}")
    print("-" * 50)

    for sid in servo_ids:
        key = str(sid)
        success, _ = ctrl.ping(sid)
        if not success:
            print(f"{sid:<5} {'--':<15} {'N/A':<10} {'N/A':<10} --")
            continue

        pos = ctrl.read_position_raw(sid)
        if pos is None:
            print(f"{sid:<5} {'--':<15} {'ERR':<10} {'ERR':<10} --")
            continue

        if key in cal:
            zero_offset = cal[key]["zero_offset"]
            name = cal[key].get("name", f"servo_{sid}")
            angle = steps_to_angle(pos - zero_offset)
            print(f"{sid:<5} {name:<15} {pos:<10} {angle:>6.1f}°    ✓")
        else:
            print(f"{sid:<5} {'--':<15} {pos:<10} {'--':<10} ✗")


def main():
    parser = argparse.ArgumentParser(
        description="Servo calibration and angle control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument("--port", "-p", help="Serial port (auto-detected)")
    parser.add_argument("--baudrate", "-b", type=int, default=BAUDRATE)
    parser.add_argument("--id", "-i", type=str, help="Servo ID(s): 5 or 5,6,7 or 5-10")
    parser.add_argument("--all", "-a", action="store_true", help="All calibrated servos")
    parser.add_argument("--calibrate", "-c", action="store_true", help="Save current positions as zero")
    parser.add_argument("--angle", type=float, help="Target angle in degrees")
    parser.add_argument("--status", "-s", action="store_true", help="Show current status")

    args = parser.parse_args()

    # Auto-detect port
    port = args.port or auto_detect_port()
    if not port:
        print("Error: No serial port found")
        sys.exit(1)

    print(f"Port: {port}")

    # Create controller using unified module
    ctrl = ServoController(port, args.baudrate)
    if not ctrl.connect():
        print(f"Error: Failed to open {port}")
        sys.exit(1)

    try:
        # Parse servo IDs
        servo_ids = []
        if args.id:
            for part in args.id.split(","):
                if "-" in part:
                    start, end = map(int, part.split("-"))
                    servo_ids.extend(range(start, end + 1))
                else:
                    servo_ids.append(int(part))
        elif args.all:
            cal = load_calibration()
            servo_ids = [int(k) for k in cal.keys()]
        else:
            # Default: all arm servos
            servo_ids = [5, 6, 7, 8, 9, 10]

        if not servo_ids:
            print("No servos specified. Use --id or --all")
            sys.exit(1)

        print(f"Servos: {servo_ids}")

        # Execute command
        if args.calibrate:
            calibrate(ctrl, servo_ids)
        elif args.angle is not None:
            move_to_angle(ctrl, servo_ids, args.angle)
        elif args.status:
            show_status(ctrl, servo_ids)
        else:
            show_status(ctrl, servo_ids)

    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
