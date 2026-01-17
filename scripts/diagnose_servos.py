#!/usr/bin/env python3
"""
Servo Position Diagnostic Tool
===============================
Interactive script to log servo encoder positions on keypress.
Helps understand physical arm positions (abduction, extension, etc.)

Usage:
  1. Connect Waveshare adapter via USB (jumper B)
  2. Run: python scripts/diagnose_servos.py
  3. Move robot arms to different positions
  4. Press keys to log positions with annotations

Controls:
  SPACE - Log current positions with custom annotation
  n     - Log as "neutral" position
  a     - Log as "abducted" (arms out to sides)
  e     - Log as "extended" (arms forward)
  b     - Log as "back" (arms behind)
  u     - Log as "up" (arms raised)
  d     - Log as "down" (arms lowered)
  q     - Quit

Output saved to: logs/servo_positions_<timestamp>.txt
"""

import os
import sys
import json
import time
from datetime import datetime

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.servo_controller import ServoController, auto_detect_port, DEFAULT_BAUDRATE as BAUDRATE

# Servo configuration
SERVO_IDS = [5, 6, 7, 8, 9, 10]
SERVO_NAMES = {
    5: "R_Shoulder1 (ext)",
    6: "R_Shoulder2 (abd)",
    7: "R_Elbow",
    8: "L_Shoulder1 (ext)",
    9: "L_Shoulder2 (abd)",
    10: "L_Elbow"
}


def setup_terminal():
    """Set terminal to raw mode for single keypress reading."""
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old_settings


def restore_terminal(old_settings):
    """Restore terminal settings."""
    import termios

    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def get_key():
    """Read a single keypress."""
    import select

    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None


def read_all_positions(ctrl):
    """Read all servo positions."""
    positions = {}
    for sid in SERVO_IDS:
        pos = ctrl.read_position_raw(sid)
        positions[sid] = pos
        time.sleep(0.005)  # Small delay between reads
    return positions


def main():
    # Find serial port
    device = auto_detect_port()
    if device is None:
        print("ERROR: No USB serial adapter found.")
        print("       Connect your Waveshare adapter (jumper B) and try again.")
        print("       Looking for: /dev/tty.usbserial-*, /dev/ttyUSB*, /dev/ttyACM*")
        return
    print(f"Using serial port: {device}")

    # Create output directory
    log_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)

    # Create log file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"servo_positions_{timestamp}.txt")

    print("\n" + "="*70)
    print("  SERVO POSITION DIAGNOSTIC TOOL")
    print("="*70)
    print("""
Controls:
  SPACE - Log with custom annotation
  n     - Log as "neutral"
  a     - Log as "abducted" (arms out to sides)
  e     - Log as "extended" (arms forward)
  b     - Log as "back" (arms behind)
  u     - Log as "up" (arms raised)
  d     - Log as "down" (arms lowered)
  q     - Quit
""")
    print(f"Output: {log_file}")
    print("="*70 + "\n")

    # Connect to servos using unified controller
    ctrl = ServoController(device, BAUDRATE)
    if not ctrl.connect():
        print(f"ERROR: Failed to connect to {device}")
        return

    # Annotation shortcuts
    annotations = {
        'n': "neutral",
        'a': "abducted",
        'e': "extended",
        'b': "back",
        'u': "up",
        'd': "down",
    }

    # Open log file
    with open(log_file, 'w') as f:
        f.write("="*70 + "\n")
        f.write(f"  Servo Position Log - {timestamp}\n")
        f.write("="*70 + "\n\n")
        f.write("Servo IDs:\n")
        for sid, name in SERVO_NAMES.items():
            f.write(f"  {sid}: {name}\n")
        f.write("\n" + "-"*70 + "\n")
        f.write("Entry │  ID 5  │  ID 6  │  ID 7  │  ID 8  │  ID 9  │ ID 10  │ Annotation\n")
        f.write("-"*70 + "\n")

        entry_num = 0
        old_settings = setup_terminal()

        try:
            print("Waiting for keypress... (press 'q' to quit)\n")

            while True:
                key = get_key()

                if key is None:
                    continue

                if key == 'q':
                    print("\nQuitting...")
                    break

                # Determine annotation
                if key == ' ':
                    restore_terminal(old_settings)
                    annotation = input("Enter annotation: ").strip()
                    old_settings = setup_terminal()
                elif key in annotations:
                    annotation = annotations[key]
                else:
                    continue  # Unknown key

                # Read positions
                positions = read_all_positions(ctrl)
                entry_num += 1

                # Format output
                pos_strs = []
                for sid in SERVO_IDS:
                    pos = positions.get(sid)
                    if pos is not None:
                        pos_strs.append(f"{pos:6d}")
                    else:
                        pos_strs.append("  FAIL")

                line = f"{entry_num:5d} │ {' │ '.join(pos_strs)} │ {annotation}"

                # Print to screen
                print(line)

                # Write to file
                f.write(line + "\n")
                f.flush()

        finally:
            restore_terminal(old_settings)
            ctrl.disconnect()

    print(f"\nLog saved to: {log_file}")

    # Load and display current calibration for comparison
    cal_file = os.path.join(os.path.dirname(__file__), "servo_calibration.json")
    if os.path.exists(cal_file):
        with open(cal_file, 'r') as f:
            cal = json.load(f)

        print("\n" + "="*70)
        print("  Current Calibration (zero_offset values):")
        print("-"*70)
        for sid in SERVO_IDS:
            sid_str = str(sid)
            if sid_str in cal:
                offset = cal[sid_str]["zero_offset"]
                name = cal[sid_str].get("name", SERVO_NAMES.get(sid, ""))
                print(f"  ID {sid}: {offset:6d}  ({name})")
        print("="*70)


if __name__ == "__main__":
    main()
