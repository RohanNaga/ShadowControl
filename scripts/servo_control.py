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

# Config file for zero offsets
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "servo_calibration.json")

# STS3215 constants
ADDR_GOAL_POSITION = 42
ADDR_CURRENT_POSITION = 56
STEPS_PER_REV = 4096  # 12-bit encoder
BAUDRATE = 500000

# Try to import servo SDK
try:
    from scservo_sdk import PortHandler, PacketHandler
except ImportError:
    try:
        from feetech_servo_sdk import PortHandler, PacketHandler
    except ImportError:
        PortHandler = None
        PacketHandler = None


class RawServoController:
    """Raw serial controller for Milk-V (no SDK required)."""

    def __init__(self, port="/dev/ttyS2", baudrate=500000):
        self.port = port
        self.baudrate = baudrate
        self.fd = None

    def open(self):
        os.system(f"stty -F {self.port} {self.baudrate} raw -echo cs8 -cstopb -parenb 2>/dev/null")
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        return True

    def close(self):
        if self.fd:
            os.close(self.fd)

    def _checksum(self, data):
        return (~sum(data)) & 0xFF

    def _flush(self):
        """Flush any pending data."""
        try:
            while True:
                data = os.read(self.fd, 256)
                if not data:
                    break
        except:
            pass

    def ping(self, servo_id):
        self._flush()
        packet = bytes([0xFF, 0xFF, servo_id, 0x02, 0x01])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.02)
        try:
            resp = os.read(self.fd, 64)
            if resp and len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
                return True, resp[4]
        except:
            pass
        return False, None

    def read_position(self, servo_id):
        self._flush()
        packet = bytes([0xFF, 0xFF, servo_id, 0x04, 0x02, ADDR_CURRENT_POSITION, 2])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.02)
        try:
            resp = os.read(self.fd, 64)
            if resp and len(resp) >= 8:
                return resp[5] | (resp[6] << 8)
        except:
            pass
        return None

    def write_position(self, servo_id, position):
        position = int(position) & 0xFFFF  # Clamp to 16-bit
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        packet = bytes([0xFF, 0xFF, servo_id, 0x05, 0x03, ADDR_GOAL_POSITION, pos_l, pos_h])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.01)


class SDKServoController:
    """SDK-based controller for PC."""

    def __init__(self, port, baudrate=500000):
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(1)
        self.baudrate = baudrate

    def open(self):
        if not self.port_handler.openPort():
            return False
        self.port_handler.setBaudRate(self.baudrate)
        return True

    def close(self):
        self.port_handler.closePort()

    def ping(self, servo_id):
        model, result, error = self.packet_handler.ping(self.port_handler, servo_id)
        return result == 0, error

    def read_position(self, servo_id):
        pos, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, ADDR_CURRENT_POSITION
        )
        return pos if result == 0 else None

    def write_position(self, servo_id, position):
        position = int(position) & 0xFFFF
        self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, ADDR_GOAL_POSITION, position
        )


def auto_detect_port():
    """Auto-detect serial port."""
    import glob

    # Milk-V UART2
    if os.path.exists("/dev/ttyS2"):
        return "/dev/ttyS2"

    # macOS USB
    ports = glob.glob("/dev/tty.usbmodem*")
    if ports:
        return ports[0]

    # Linux USB
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    return None


def create_controller(port, baudrate):
    """Create appropriate controller based on environment."""
    if PortHandler is None or port == "/dev/ttyS2":
        return RawServoController(port, baudrate)
    else:
        return SDKServoController(port, baudrate)


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


def angle_to_steps(angle):
    """Convert angle (degrees) to steps."""
    return int(angle * STEPS_PER_REV / 360)


def steps_to_angle(steps):
    """Convert steps to angle (degrees)."""
    return steps * 360 / STEPS_PER_REV


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

        pos = ctrl.read_position(sid)
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

        ctrl.write_position(sid, target_steps)
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

        pos = ctrl.read_position(sid)
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
    parser.add_argument("--baudrate", "-b", type=int, default=500000)
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

    # Create controller
    ctrl = create_controller(port, args.baudrate)
    if not ctrl.open():
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
        ctrl.close()


if __name__ == "__main__":
    main()
