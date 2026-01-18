#!/usr/bin/env python3
"""
Test Feetech STS3215 servo communication and movement.

This script can run on:
- Milk-V Duo S (UART mode, jumper A) - use /dev/ttyS2 at 500kbps
- PC with Waveshare USB (jumper B) - auto-detects port

Usage:
    # Scan for servos
    python test_servo.py --scan

    # Test specific servo
    python test_servo.py --id 5

    # Test multiple servos
    python test_servo.py --id 5,6,7

    # Test all servos found
    python test_servo.py --all

    # Move servo to specific position
    python test_servo.py --id 5 --position 2048

    # Oscillate servo
    python test_servo.py --id 5 --oscillate
"""

import argparse
import sys
import time
import os

# Add parent directory for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.servo_controller import (
    ServoController,
    auto_detect_port,
    scan_servos,
    DEFAULT_BAUDRATE as BAUDRATE,
)


def test_servo(ctrl, servo_id, oscillate=False, position=None):
    """Test a single servo."""
    print(f"\n--- Servo ID {servo_id} ---")

    # Ping
    success, error = ctrl.ping(servo_id)
    if not success:
        print(f"  ✗ Not responding")
        return False

    print(f"  ✓ Responding")

    # Read position
    pos = ctrl.read_position_raw(servo_id)
    if pos is None:
        print(f"  ✗ Failed to read position")
        return False

    print(f"  Position: {pos}")

    # Move to specific position
    if position is not None:
        print(f"  Moving to {position}...")
        ctrl.write_position_raw(servo_id, position)
        time.sleep(1)
        new_pos = ctrl.read_position_raw(servo_id)
        print(f"  New position: {new_pos}")

    # Oscillate
    if oscillate:
        # Read position 3 times to check stability
        print(f"  Reading position 3 times...")
        for i in range(3):
            p = ctrl.read_position_raw(servo_id)
            time.sleep(0.1)
            print(f"    Read {i+1}: {p}")

        step = 10  # ~0.9 degrees
        print(f"  Oscillating ±{step} steps (~{step * 360 / 4096:.1f}°)...")
        original_pos = pos
        for i in range(2):
            target_plus = original_pos + step
            ctrl.write_position_raw(servo_id, target_plus)
            time.sleep(0.5)  # longer delay
            actual = ctrl.read_position_raw(servo_id)
            print(f"    +{step}: target={target_plus}, actual={actual}")

            target_minus = original_pos - step
            ctrl.write_position_raw(servo_id, target_minus)
            time.sleep(0.5)
            actual = ctrl.read_position_raw(servo_id)
            print(f"    -{step}: target={target_minus}, actual={actual}")

        ctrl.write_position_raw(servo_id, original_pos)
        time.sleep(0.3)
        final = ctrl.read_position_raw(servo_id)
        print(f"  Return to {original_pos}, final={final}")
        print(f"  ✓ Oscillation complete")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Test Feetech STS3215 servos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument("--port", "-p", help="Serial port (auto-detected)")
    parser.add_argument("--baudrate", "-b", type=int, default=BAUDRATE, help="Baudrate (default: 500000)")
    parser.add_argument("--scan", "-s", action="store_true", help="Scan for servos")
    parser.add_argument("--id", "-i", type=str, help="Servo ID(s) to test (e.g., 5 or 5,6,7)")
    parser.add_argument("--all", "-a", action="store_true", help="Test all found servos")
    parser.add_argument("--oscillate", "-o", action="store_true", help="Oscillate servo")
    parser.add_argument("--position", type=int, help="Move to specific position (0-4095)")

    args = parser.parse_args()

    # Auto-detect port
    port = args.port or auto_detect_port()
    if not port:
        print("Error: No serial port found")
        sys.exit(1)

    print(f"Port: {port}")
    print(f"Baudrate: {args.baudrate}")

    # Create controller using unified module
    ctrl = ServoController(port, args.baudrate)

    if not ctrl.connect():
        print(f"Error: Failed to open {port}")
        sys.exit(1)

    try:
        if args.scan or args.all:
            print("\n" + "=" * 40)
            print("SCANNING FOR SERVOS")
            print("=" * 40)
            found = scan_servos(ctrl)

            if not found:
                print("  No servos found!")
            else:
                print(f"\nFound {len(found)} servo(s)")

            if args.all and found:
                print("\n" + "=" * 40)
                print("TESTING ALL SERVOS")
                print("=" * 40)
                for sid, _ in found:
                    test_servo(ctrl, sid, oscillate=args.oscillate, position=args.position)

        elif args.id:
            # Parse IDs
            ids = []
            for part in args.id.split(","):
                if "-" in part:
                    start, end = map(int, part.split("-"))
                    ids.extend(range(start, end + 1))
                else:
                    ids.append(int(part))

            print("\n" + "=" * 40)
            print(f"TESTING SERVO(S): {ids}")
            print("=" * 40)

            for sid in ids:
                test_servo(ctrl, sid, oscillate=args.oscillate, position=args.position)

        else:
            print("No action specified. Use --scan, --id, or --all")
            print("Run with --help for usage.")

    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
