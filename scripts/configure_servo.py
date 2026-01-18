#!/usr/bin/env python3
"""
Configure Feetech STS3215 servo: set ID and baudrate.

This script runs on a PC with the Waveshare adapter in USB mode (jumper B).
Connect ONE servo at a time to configure.

Usage:
    # Configure servo to ID 5 at 500kbps
    python scripts/configure_servo.py --new-id 5

    # Configure with specific port
    python scripts/configure_servo.py --new-id 6 --port /dev/tty.usbmodem5A7C1165541

    # Scan for all servos
    python scripts/configure_servo.py --scan

    # Test movement of a specific servo
    python scripts/configure_servo.py --test 5

Requirements:
    pip install feetech-servo-sdk
"""

import argparse
import sys
import time

try:
    from scservo_sdk import PortHandler, PacketHandler
except ImportError:
    try:
        from feetech_servo_sdk import PortHandler, PacketHandler
    except ImportError:
        print("Error: feetech-servo-sdk not installed")
        print("Install with: pip install feetech-servo-sdk")
        sys.exit(1)


# STS3215 Register Addresses (EEPROM)
ADDR_ID = 5
ADDR_BAUDRATE = 6
ADDR_EEPROM_LOCK = 55

# STS3215 Register Addresses (SRAM)
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_CURRENT_POSITION = 56
ADDR_CURRENT_SPEED = 58
ADDR_CURRENT_LOAD = 60

# Baudrate values
BAUDRATES = {
    0: 1000000,
    1: 500000,
    2: 250000,
    3: 128000,
    4: 115200,
}

TARGET_BAUDRATE = 500000
TARGET_BAUD_VALUE = 1


def find_servo(port, packet, factory_only=False):
    """Find servo at any baudrate, return (id, baudrate, model) or (None, None, None).

    If factory_only=True, only scan at 1Mbps for unconfigured servos.
    """
    baudrates = [1000000] if factory_only else [1000000, 500000, 250000, 115200]

    for baud in baudrates:
        port.setBaudRate(baud)
        for sid in range(1, 51):
            model, result, _ = packet.ping(port, sid)
            if result == 0:
                actual_id = sid if sid != 254 else 1
                return actual_id, baud, model
    return None, None, None


def scan_all(port, packet):
    """Scan for all servos at all baudrates."""
    print("=" * 50)
    print("SCANNING FOR SERVOS")
    print("=" * 50)

    found = []
    for baud in [1000000, 500000]:
        port.setBaudRate(baud)
        print(f"\n{baud} bps:")
        for sid in range(1, 51):
            model, result, _ = packet.ping(port, sid)
            if result == 0:
                print(f"  ID {sid}: model {model}")
                found.append((sid, baud, model))

    if not found:
        print("\nNo servos found!")
    else:
        print(f"\nTotal: {len(found)} servo(s)")

    return found


def configure_servo(port, packet, new_id, factory_only=False):
    """Configure servo to new ID and 500kbps baudrate.

    If factory_only=True, only look for unconfigured servos at 1Mbps.
    """
    print("=" * 50)
    print(f"CONFIGURING SERVO → ID {new_id} @ 500kbps")
    if factory_only:
        print("(Factory mode: only scanning 1Mbps)")
    print("=" * 50)

    # Find servo
    print("\n1. Scanning for servo...")
    old_id, baud, model = find_servo(port, packet, factory_only=factory_only)

    if old_id is None:
        print("   No servo found! Check connection.")
        return False

    print(f"   Found: ID {old_id} @ {baud} bps (model {model})")

    # Check if already configured
    if old_id == new_id and baud == TARGET_BAUDRATE:
        print(f"\n   Servo already configured as ID {new_id} @ 500kbps!")
        return True

    # Unlock EEPROM
    print("\n2. Unlocking EEPROM...")
    result, error = packet.write1ByteTxRx(port, old_id, ADDR_EEPROM_LOCK, 0)
    print(f"   Result: {result}, Error: {error}")
    time.sleep(0.3)

    # Track which ID to use for subsequent commands
    # ID change takes effect IMMEDIATELY, baudrate requires power cycle
    current_id = old_id

    # Set ID FIRST (takes effect immediately!)
    if old_id != new_id:
        print(f"\n3. Setting ID to {new_id}...")
        result, error = packet.write1ByteTxRx(port, old_id, ADDR_ID, new_id)
        print(f"   Result: {result}, Error: {error}")
        time.sleep(0.3)
        current_id = new_id  # Servo now responds at new ID!
        print(f"   (Servo now responds at ID {new_id})")
    else:
        print(f"\n3. ID already {new_id}, skipping...")

    # Set baudrate if needed (use current_id since ID change is immediate)
    if baud != TARGET_BAUDRATE:
        print(f"\n4. Setting baudrate to 500kbps (writing to ID {current_id})...")
        result, error = packet.write1ByteTxRx(port, current_id, ADDR_BAUDRATE, TARGET_BAUD_VALUE)
        print(f"   Result: {result}, Error: {error}")
        time.sleep(0.3)
    else:
        print(f"\n4. Baudrate already 500kbps, skipping...")

    # Lock EEPROM (use current_id)
    print(f"\n5. Locking EEPROM (writing to ID {current_id})...")
    result, error = packet.write1ByteTxRx(port, current_id, ADDR_EEPROM_LOCK, 1)
    print(f"   Result: {result}, Error: {error}")
    time.sleep(0.3)

    # Verify
    print("\n6. Verifying...")
    port.setBaudRate(TARGET_BAUDRATE)
    time.sleep(0.2)

    model, result, _ = packet.ping(port, new_id)
    if result == 0:
        print(f"   ✓ SUCCESS! Servo responding as ID {new_id} @ 500kbps")
        return True
    else:
        print(f"   ⚠ Verification failed - power cycle Waveshare and run --test {new_id}")
        return False


SERVO_NAMES = {
    5: "R_Shoulder1",
    6: "R_Shoulder2",
    7: "R_Elbow",
    8: "L_Shoulder1",
    9: "L_Shoulder2",
    10: "L_Elbow",
}

NEUTRAL_POSITIONS = {
    5: 2048,   # R_Shoulder1
    6: 2048,   # R_Shoulder2
    7: 2048,   # R_Elbow
    8: 2048,   # L_Shoulder1
    9: 2048,   # L_Shoulder2
    10: 2048,  # L_Elbow
}


def read_zero_positions(port, packet):
    """Read current positions of all servos (for calibration)."""
    print("=" * 50)
    print("READING ZERO POSITIONS")
    print("=" * 50)

    port.setBaudRate(TARGET_BAUDRATE)

    positions = {}
    print("\nCurrent servo positions:")
    print("-" * 50)

    for sid in [5, 6, 7, 8, 9, 10]:
        _, result, _ = packet.ping(port, sid)
        if result != 0:
            print(f"  ID {sid} ({SERVO_NAMES[sid]}): OFFLINE")
            continue

        pos, result, _ = packet.read2ByteTxRx(port, sid, ADDR_CURRENT_POSITION)
        if result != 0:
            print(f"  ID {sid} ({SERVO_NAMES[sid]}): READ FAILED")
            continue

        # Swap bytes (SDK endianness fix)
        pos = ((pos & 0xFF) << 8) | ((pos >> 8) & 0xFF)
        positions[sid] = pos
        print(f"  ID {sid} ({SERVO_NAMES[sid]}): {pos}")

    print("-" * 50)
    print("\nCopy this for NEUTRAL_POSITIONS or calibration:")
    print("{")
    for sid, pos in positions.items():
        print(f"    {sid}: {pos},  # {SERVO_NAMES[sid]}")
    print("}")

    return positions


def set_zero_positions(port, packet):
    """Move all servos to their neutral/zero positions."""
    print("=" * 50)
    print("SETTING SERVOS TO NEUTRAL POSITIONS")
    print("=" * 50)

    port.setBaudRate(TARGET_BAUDRATE)

    # Set speed first (servos need non-zero speed to move)
    print("\nSetting servo speeds...")
    for sid in NEUTRAL_POSITIONS.keys():
        speed = 2000
        speed_swapped = ((speed & 0xFF) << 8) | ((speed >> 8) & 0xFF)
        packet.write2ByteTxRx(port, sid, ADDR_GOAL_SPEED, speed_swapped)
        time.sleep(0.01)

    print("\nMoving to neutral positions:")
    print("-" * 50)

    for sid, target in NEUTRAL_POSITIONS.items():
        _, result, _ = packet.ping(port, sid)
        if result != 0:
            print(f"  ID {sid} ({SERVO_NAMES[sid]}): OFFLINE - skipped")
            continue

        # Swap bytes for SDK
        target_swapped = ((target & 0xFF) << 8) | ((target >> 8) & 0xFF)
        packet.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, target_swapped)
        print(f"  ID {sid} ({SERVO_NAMES[sid]}): -> {target}")
        time.sleep(0.1)

    print("-" * 50)
    print("Done! Servos moved to neutral positions.")


def test_servo(port, packet, servo_id):
    """Test servo movement."""
    print("=" * 50)
    print(f"TESTING SERVO ID {servo_id}")
    print("=" * 50)

    port.setBaudRate(TARGET_BAUDRATE)

    # Ping
    print("\n1. Pinging...")
    model, result, _ = packet.ping(port, servo_id)
    if result != 0:
        print(f"   ✗ Servo ID {servo_id} not responding at 500kbps")
        return False

    print(f"   ✓ Found servo ID {servo_id} (model {model})")

    # Read position
    print("\n2. Reading current position...")
    pos, result, _ = packet.read2ByteTxRx(port, servo_id, ADDR_CURRENT_POSITION)
    if result != 0:
        print("   ✗ Failed to read position")
        return False

    print(f"   Current position: {pos}")

    # Oscillate
    print("\n3. Oscillating ±25 steps (small movement)...")
    STEP = 25
    for i in range(5):
        packet.write2ByteTxRx(port, servo_id, ADDR_GOAL_POSITION, pos + STEP)
        time.sleep(0.25)
        packet.write2ByteTxRx(port, servo_id, ADDR_GOAL_POSITION, pos - STEP)
        time.sleep(0.25)

    # Return to original
    packet.write2ByteTxRx(port, servo_id, ADDR_GOAL_POSITION, pos)
    print("   ✓ Movement test complete!")

    return True


def auto_detect_port():
    """Try to auto-detect the Waveshare USB port."""
    import glob

    # macOS
    ports = glob.glob("/dev/tty.usbmodem*")
    if ports:
        return ports[0]

    # Linux
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    return None


def main():
    parser = argparse.ArgumentParser(
        description="Configure Feetech STS3215 servo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python configure_servo.py --scan              # Scan for all servos
    python configure_servo.py --new-id 5          # Configure servo to ID 5
    python configure_servo.py --test 5            # Test servo ID 5
    python configure_servo.py --new-id 6 --test 6 # Configure and test
"""
    )

    parser.add_argument("--port", "-p", help="Serial port (auto-detected if not specified)")
    parser.add_argument("--new-id", "-n", type=int, help="New servo ID (1-253)")
    parser.add_argument("--scan", "-s", action="store_true", help="Scan for all servos")
    parser.add_argument("--test", "-t", type=int, help="Test servo movement by ID")
    parser.add_argument("--zero", "-z", action="store_true", help="Read current positions as zero/neutral")
    parser.add_argument("--set-zero", action="store_true", help="Move all servos to neutral positions")
    parser.add_argument("--factory", "-f", action="store_true",
                        help="Only configure factory servos (1Mbps) - ignores already-configured servos")

    args = parser.parse_args()

    # Auto-detect port if not specified
    port_name = args.port or auto_detect_port()
    if not port_name:
        print("Error: No serial port found. Specify with --port")
        print("  macOS: --port /dev/tty.usbmodem*")
        print("  Linux: --port /dev/ttyUSB0")
        sys.exit(1)

    # Open port
    port = PortHandler(port_name)
    packet = PacketHandler(1)

    if not port.openPort():
        print(f"Error: Failed to open {port_name}")
        print("Check:")
        print("  1. Waveshare jumper is set to B (USB mode)")
        print("  2. USB cable is connected")
        sys.exit(1)

    print(f"Opened port: {port_name}")

    try:
        if args.scan:
            scan_all(port, packet)

        if args.zero:
            read_zero_positions(port, packet)

        if args.set_zero:
            set_zero_positions(port, packet)

        if args.new_id:
            if not 1 <= args.new_id <= 253:
                print("Error: ID must be 1-253")
                sys.exit(1)
            configure_servo(port, packet, args.new_id, factory_only=args.factory)

        if args.test:
            test_servo(port, packet, args.test)

        if not args.scan and not args.new_id and not args.test and not args.zero and not args.set_zero:
            print("No action specified. Use --scan, --new-id, or --test")
            print("Run with --help for usage.")

    finally:
        port.closePort()


if __name__ == "__main__":
    main()
