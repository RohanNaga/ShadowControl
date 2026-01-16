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

# Try to import servo SDK
try:
    from scservo_sdk import PortHandler, PacketHandler
except ImportError:
    try:
        from feetech_servo_sdk import PortHandler, PacketHandler
    except ImportError:
        # Fallback to raw serial for Milk-V
        PortHandler = None
        PacketHandler = None


# STS3215 Register Addresses
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_CURRENT_POSITION = 56
ADDR_CURRENT_SPEED = 58
ADDR_CURRENT_LOAD = 60

BAUDRATE = 500000


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

    def ping(self, servo_id):
        packet = bytes([0xFF, 0xFF, servo_id, 0x02, 0x01])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.02)
        try:
            resp = os.read(self.fd, 64)
            if resp and len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
                return True, resp[4]  # success, error byte
        except:
            pass
        return False, None

    def read_position(self, servo_id):
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
        self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, ADDR_GOAL_POSITION, position
        )


def auto_detect_port():
    """Auto-detect serial port."""
    import glob

    # Check if on Milk-V (Linux with /dev/ttyS2)
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


def scan_servos(ctrl, id_range=range(1, 20)):
    """Scan for servos."""
    found = []
    for sid in id_range:
        success, _ = ctrl.ping(sid)
        if success:
            pos = ctrl.read_position(sid)
            found.append((sid, pos))
            print(f"  ID {sid}: position {pos}")
    return found


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
    pos = ctrl.read_position(servo_id)
    if pos is None:
        print(f"  ✗ Failed to read position")
        return False

    print(f"  Position: {pos}")

    # Move to specific position
    if position is not None:
        print(f"  Moving to {position}...")
        ctrl.write_position(servo_id, position)
        time.sleep(1)
        new_pos = ctrl.read_position(servo_id)
        print(f"  New position: {new_pos}")

    # Oscillate
    if oscillate:
        # Read position 3 times to check stability
        print(f"  Reading position 3 times...")
        for i in range(3):
            p = ctrl.read_position(servo_id)
            time.sleep(0.1)
            print(f"    Read {i+1}: {p}")

        step = 10  # ~0.9 degrees
        print(f"  Oscillating ±{step} steps (~{step * 360 / 4096:.1f}°)...")
        original_pos = pos
        for i in range(2):
            target_plus = original_pos + step
            ctrl.write_position(servo_id, target_plus)
            time.sleep(0.5)  # longer delay
            actual = ctrl.read_position(servo_id)
            print(f"    +{step}: target={target_plus}, actual={actual}")

            target_minus = original_pos - step
            ctrl.write_position(servo_id, target_minus)
            time.sleep(0.5)
            actual = ctrl.read_position(servo_id)
            print(f"    -{step}: target={target_minus}, actual={actual}")

        ctrl.write_position(servo_id, original_pos)
        time.sleep(0.3)
        final = ctrl.read_position(servo_id)
        print(f"  Return to {original_pos}, final={final}")
        print(f"  ✓ Oscillation complete")

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Test Feetech STS3215 servos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument("--port", "-p", help="Serial port (auto-detected)")
    parser.add_argument("--baudrate", "-b", type=int, default=500000, help="Baudrate (default: 500000)")
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

    # Create controller
    ctrl = create_controller(port, args.baudrate)

    if not ctrl.open():
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
        ctrl.close()


if __name__ == "__main__":
    main()
