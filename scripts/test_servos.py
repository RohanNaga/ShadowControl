#!/usr/bin/env python3
"""
Test servo connectivity and basic movement.

This script verifies that all servos are responding and can move to basic positions.
Run this AFTER reconfiguring servos to 500kbps baudrate.

Usage:
    python scripts/test_servos.py
    python scripts/test_servos.py --device /dev/ttyUSB0 --baudrate 500000
"""

import argparse
import sys
import time
import yaml
from pathlib import Path

# Try to import kos_zbot, fallback to feetech SDK
try:
    from kos_zbot import KosZbot
    USE_KOS = True
except ImportError:
    USE_KOS = False
    try:
        from feetech_servo_sdk import PortHandler, PacketHandler
    except ImportError:
        print("Error: Neither kos_zbot nor feetech_servo_sdk installed")
        print("Install with: pip install kos_zbot  OR  pip install feetech-servo-sdk")
        sys.exit(1)


def load_config():
    """Load robot configuration from YAML."""
    config_path = Path(__file__).parent.parent / "config" / "robot.yaml"
    if not config_path.exists():
        print(f"Warning: Config file not found at {config_path}")
        return None
    with open(config_path) as f:
        return yaml.safe_load(f)


def test_with_kos(device: str, baudrate: int, servo_ids: list):
    """Test servos using kos_zbot."""
    print(f"Testing with kos_zbot on {device} at {baudrate} bps")

    # TODO: Implement kos_zbot testing
    # This requires the actual kos_zbot API which may differ
    print("kos_zbot testing not yet implemented")
    print("Use: kos actuator move --id all --pos 0")
    return False


def test_with_feetech(device: str, baudrate: int, servo_ids: list):
    """Test servos using feetech SDK directly."""
    print(f"Testing with feetech SDK on {device} at {baudrate} bps")

    # Protocol version for STS series
    PROTOCOL_VERSION = 1

    # Control table addresses for STS3215
    ADDR_TORQUE_ENABLE = 40
    ADDR_GOAL_POSITION = 42
    ADDR_PRESENT_POSITION = 56

    port_handler = PortHandler(device)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print(f"Failed to open port {device}")
        return False

    if not port_handler.setBaudRate(baudrate):
        print(f"Failed to set baudrate to {baudrate}")
        return False

    print(f"Port opened successfully at {baudrate} bps")
    print("-" * 50)

    # Ping all servos
    responding = []
    for servo_id in servo_ids:
        model_num, result, error = packet_handler.ping(port_handler, servo_id)
        if result == 0:  # COMM_SUCCESS
            print(f"Servo {servo_id:2d}: OK (model: {model_num})")
            responding.append(servo_id)
        else:
            print(f"Servo {servo_id:2d}: NOT RESPONDING")

    print("-" * 50)
    print(f"Found {len(responding)}/{len(servo_ids)} servos")

    if not responding:
        print("\nNo servos found! Check:")
        print("  1. Servo baudrate is set to 500kbps (not 1Mbps default)")
        print("  2. Wiring: TX->TX, RX->RX (not crossed)")
        print("  3. Power supply connected to adapter")
        print("  4. Jumper set to position A (UART mode)")
        port_handler.closePort()
        return False

    # Read current positions
    print("\nCurrent positions:")
    for servo_id in responding:
        position, result, error = packet_handler.read2ByteTxRx(
            port_handler, servo_id, ADDR_PRESENT_POSITION
        )
        if result == 0:
            angle = (position - 2048) * 360 / 4096  # Convert to degrees
            print(f"  Servo {servo_id:2d}: position={position:4d} ({angle:+.1f} deg)")

    # Test movement (optional)
    test_movement = input("\nTest movement? (y/N): ").lower() == 'y'

    if test_movement:
        print("\nMoving all servos to center position (2048)...")

        # Enable torque
        for servo_id in responding:
            packet_handler.write1ByteTxRx(
                port_handler, servo_id, ADDR_TORQUE_ENABLE, 1
            )

        # Move to center
        for servo_id in responding:
            packet_handler.write2ByteTxRx(
                port_handler, servo_id, ADDR_GOAL_POSITION, 2048
            )

        time.sleep(1)

        print("Movement complete. Current positions:")
        for servo_id in responding:
            position, result, error = packet_handler.read2ByteTxRx(
                port_handler, servo_id, ADDR_PRESENT_POSITION
            )
            if result == 0:
                print(f"  Servo {servo_id:2d}: {position}")

        # Disable torque
        for servo_id in responding:
            packet_handler.write1ByteTxRx(
                port_handler, servo_id, ADDR_TORQUE_ENABLE, 0
            )

    port_handler.closePort()
    return True


def main():
    parser = argparse.ArgumentParser(description="Test servo connectivity")
    parser.add_argument(
        "--device", "-d",
        default="/dev/ttyAMA5",
        help="Serial device (default: /dev/ttyAMA5)"
    )
    parser.add_argument(
        "--baudrate", "-b",
        type=int,
        default=500000,
        help="Baudrate (default: 500000)"
    )
    parser.add_argument(
        "--ids",
        type=str,
        default="1-16",
        help="Servo IDs to test (default: 1-16)"
    )
    args = parser.parse_args()

    # Parse servo IDs
    servo_ids = []
    for part in args.ids.split(","):
        if "-" in part:
            start, end = map(int, part.split("-"))
            servo_ids.extend(range(start, end + 1))
        else:
            servo_ids.append(int(part))

    print("=" * 50)
    print("Shadow Control - Servo Test")
    print("=" * 50)
    print(f"Device:   {args.device}")
    print(f"Baudrate: {args.baudrate}")
    print(f"Servo IDs: {servo_ids}")
    print("=" * 50)

    # Load config if available
    config = load_config()
    if config:
        print(f"Config loaded: {config['robot']['name']}")

    # Test with appropriate library
    if USE_KOS:
        success = test_with_kos(args.device, args.baudrate, servo_ids)
    else:
        success = test_with_feetech(args.device, args.baudrate, servo_ids)

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
