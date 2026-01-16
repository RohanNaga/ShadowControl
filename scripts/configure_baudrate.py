#!/usr/bin/env python3
"""
Reconfigure Feetech STS3215 servo baudrate from 1Mbps to 500kbps.

This script runs on a PC with the Waveshare adapter in USB mode (jumper B).
Connect ONE servo at a time to reconfigure its baudrate.

CRITICAL: The Milk-V Duo S UART max is 921,600 bps, but servos default to 1Mbps.
We must reconfigure each servo to 500kbps before they can communicate with Milk-V.

Usage:
    # Single servo
    python scripts/configure_baudrate.py --id 1

    # Range of servos (connect one at a time)
    python scripts/configure_baudrate.py --id 1-16 --interactive

    # Scan for any servo at factory baudrate
    python scripts/configure_baudrate.py --scan

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


# Feetech STS3215 register addresses (EEPROM)
ADDR_BAUDRATE = 6       # Baudrate register (1 byte) - was incorrectly 4
ADDR_ID = 5             # Servo ID register (1 byte)
ADDR_EEPROM_LOCK = 55   # EEPROM lock: 0=unlock, 1=lock

# Feetech STS3215 register addresses (SRAM)
ADDR_TORQUE_ENABLE = 40  # 0x28
ADDR_GOAL_POSITION = 42  # 0x2A (2 bytes)
ADDR_GOAL_SPEED = 46     # 0x2E (2 bytes)
ADDR_CURRENT_POSITION = 56  # 0x38 (2 bytes, read-only)

# Baudrate values for register
BAUDRATE_VALUES = {
    1000000: 0,  # Factory default
    500000: 1,   # Target for Milk-V
    250000: 2,
    128000: 3,
    115200: 4,
}

# Protocol version
PROTOCOL_VERSION = 1


def scan_servo(port_handler, packet_handler, baudrate, broadcast=False):
    """Scan for servos at given baudrate."""
    port_handler.setBaudRate(baudrate)
    found = []

    # Scan ID range (broadcast ID 254 or individual IDs)
    ids_to_scan = [254] if broadcast else range(1, 254)

    for servo_id in ids_to_scan:
        model_num, result, error = packet_handler.ping(port_handler, servo_id)
        if result == 0:  # COMM_SUCCESS
            found.append((servo_id, model_num))
            if servo_id == 254:
                print(f"  Found servo responding to broadcast at {baudrate} bps")
            else:
                print(f"  Found servo ID {servo_id} (model: {model_num}) at {baudrate} bps")

    return found


def change_baudrate(port_handler, packet_handler, servo_id, current_baud, target_baud):
    """Change servo baudrate from current to target."""
    # Connect at current baudrate
    port_handler.setBaudRate(current_baud)

    # Ping to verify connection
    model_num, result, error = packet_handler.ping(port_handler, servo_id)
    if result != 0:
        print(f"  Cannot communicate with servo {servo_id} at {current_baud} bps")
        return False

    print(f"  Connected to servo {servo_id} (model: {model_num}) at {current_baud} bps")

    # Get target baudrate register value
    if target_baud not in BAUDRATE_VALUES:
        print(f"  Invalid target baudrate: {target_baud}")
        return False

    target_value = BAUDRATE_VALUES[target_baud]

    # Unlock EEPROM (write 0 to register 55)
    print(f"  Unlocking EEPROM...")
    packet_handler.write1ByteTxRx(port_handler, servo_id, ADDR_EEPROM_LOCK, 0)
    time.sleep(0.1)

    # Write new baudrate value
    result, error = packet_handler.write1ByteTxRx(
        port_handler, servo_id, ADDR_BAUDRATE, target_value
    )

    if result != 0:
        print(f"  Failed to write baudrate register: error {result}")
        return False

    print(f"  Wrote baudrate register value {target_value} (={target_baud} bps)")

    # Lock EEPROM (write 1 to register 55)
    print(f"  Locking EEPROM...")
    packet_handler.write1ByteTxRx(port_handler, servo_id, ADDR_EEPROM_LOCK, 1)
    time.sleep(0.1)

    # Wait for servo to apply change
    time.sleep(0.5)

    # Verify by connecting at new baudrate
    port_handler.setBaudRate(target_baud)
    time.sleep(0.1)

    model_num, result, error = packet_handler.ping(port_handler, servo_id)
    if result == 0:
        print(f"  SUCCESS: Servo {servo_id} now responding at {target_baud} bps!")
        return True
    else:
        print(f"  WARNING: Servo {servo_id} not responding at new baudrate")
        print(f"           Power cycle the servo and try again")
        return False


def interactive_mode(port_handler, packet_handler, servo_ids, current_baud, target_baud):
    """Interactively configure servos one at a time."""
    print("\n" + "=" * 60)
    print("INTERACTIVE BAUDRATE CONFIGURATION")
    print("=" * 60)
    print(f"Servos to configure: {servo_ids}")
    print(f"Current baudrate: {current_baud} bps")
    print(f"Target baudrate:  {target_baud} bps")
    print("=" * 60)

    success_count = 0
    fail_count = 0

    for servo_id in servo_ids:
        print(f"\n--- Servo ID {servo_id} ---")
        print(f"1. Connect servo ID {servo_id} to the adapter")
        print(f"2. Make sure only this ONE servo is connected")
        input(f"Press Enter when ready (or Ctrl+C to skip)...")

        try:
            if change_baudrate(port_handler, packet_handler, servo_id, current_baud, target_baud):
                success_count += 1
            else:
                fail_count += 1
        except KeyboardInterrupt:
            print("\n  Skipped")
            continue

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Successfully configured: {success_count}")
    print(f"Failed: {fail_count}")

    return fail_count == 0


def batch_mode(port_handler, packet_handler, servo_ids, current_baud, target_baud):
    """Configure all connected servos (assumes they're on different IDs)."""
    print("\n" + "=" * 60)
    print("BATCH BAUDRATE CONFIGURATION")
    print("=" * 60)
    print(f"WARNING: All servos must have UNIQUE IDs and be connected")
    print(f"Servos to configure: {servo_ids}")
    print("=" * 60)

    success_count = 0
    fail_count = 0

    for servo_id in servo_ids:
        print(f"\n--- Servo ID {servo_id} ---")
        if change_baudrate(port_handler, packet_handler, servo_id, current_baud, target_baud):
            success_count += 1
        else:
            fail_count += 1

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Successfully configured: {success_count}")
    print(f"Failed: {fail_count}")

    return fail_count == 0


def main():
    parser = argparse.ArgumentParser(
        description="Reconfigure Feetech servo baudrate for Milk-V compatibility",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Scan for servos at all baudrates
    python configure_baudrate.py --scan

    # Configure servo ID 1
    python configure_baudrate.py --id 1

    # Configure servos 1-16 interactively (one at a time)
    python configure_baudrate.py --id 1-16 --interactive

    # Configure specific IDs
    python configure_baudrate.py --id 1,3,5,7

    # Use different port (Windows)
    python configure_baudrate.py --port COM3 --id 1

Baudrate Register Values:
    0 = 1,000,000 bps (factory default)
    1 = 500,000 bps   (Milk-V compatible)
    2 = 250,000 bps
    3 = 128,000 bps
    4 = 115,200 bps
"""
    )

    parser.add_argument(
        "--port", "-p",
        default="/dev/ttyUSB0",
        help="Serial port (default: /dev/ttyUSB0, use COM3 on Windows)"
    )
    parser.add_argument(
        "--id", "-i",
        type=str,
        help="Servo ID(s) to configure. Examples: 1, 1-16, 1,3,5"
    )
    parser.add_argument(
        "--scan", "-s",
        action="store_true",
        help="Scan for servos at all baudrates"
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Interactive mode: prompt for each servo"
    )
    parser.add_argument(
        "--from-baud",
        type=int,
        default=1000000,
        help="Current baudrate (default: 1000000)"
    )
    parser.add_argument(
        "--to-baud",
        type=int,
        default=500000,
        help="Target baudrate (default: 500000)"
    )

    args = parser.parse_args()

    # Initialize port handler
    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"Failed to open port {args.port}")
        print("Check:")
        print("  1. Waveshare adapter jumper is set to B (USB mode)")
        print("  2. Adapter is connected via USB")
        print("  3. Correct port name (try 'ls /dev/ttyUSB*' or check Device Manager)")
        sys.exit(1)

    print(f"Opened port {args.port}")

    try:
        if args.scan:
            print("\n" + "=" * 60)
            print("SCANNING FOR SERVOS")
            print("=" * 60)

            for baudrate in [1000000, 500000, 250000, 115200]:
                print(f"\nScanning at {baudrate} bps...")
                found = scan_servo(port_handler, packet_handler, baudrate)
                if not found:
                    print("  (none found)")

            sys.exit(0)

        if not args.id:
            print("Error: --id required (or use --scan)")
            sys.exit(1)

        # Parse servo IDs
        servo_ids = []
        for part in args.id.split(","):
            if "-" in part:
                start, end = map(int, part.split("-"))
                servo_ids.extend(range(start, end + 1))
            else:
                servo_ids.append(int(part))

        print(f"Servo IDs: {servo_ids}")

        if args.interactive:
            success = interactive_mode(
                port_handler, packet_handler, servo_ids,
                args.from_baud, args.to_baud
            )
        else:
            success = batch_mode(
                port_handler, packet_handler, servo_ids,
                args.from_baud, args.to_baud
            )

        sys.exit(0 if success else 1)

    finally:
        port_handler.closePort()


if __name__ == "__main__":
    main()
