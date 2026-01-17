#!/usr/bin/env python3
"""
Unified Servo Controller Module for ShadowControl

Provides servo communication for both PC (via USB adapter) and Milk-V (via UART).
Supports Feetech STS3215 servos with 12-bit encoders (4096 steps/revolution).

Usage:
    from src.servo_controller import ServoController, auto_detect_port

    port = auto_detect_port()
    ctrl = ServoController(port)
    ctrl.connect()
    ctrl.load_calibration("scripts/servo_calibration.json")

    # Write angle (relative to calibrated zero)
    ctrl.write_angle(5, 45.0)

    # Read current angle
    angle = ctrl.read_angle(5)

    # Sync write multiple servos
    ctrl.write_angles({5: 30.0, 6: 45.0, 8: 30.0, 9: 45.0})

    ctrl.disconnect()
"""

import glob
import json
import os
import time

# STS3215 constants
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_TIME = 44
ADDR_GOAL_SPEED = 46
ADDR_CURRENT_POSITION = 56
ADDR_CURRENT_SPEED = 58
ADDR_CURRENT_LOAD = 60

DEFAULT_SERVO_SPEED = 2000  # Steps/second (0 means "don't move", not "max speed")

STEPS_PER_REV = 4096  # 12-bit encoder
DEFAULT_BAUDRATE = 500000

# Try to import servo SDK
try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    from scservo_sdk.group_sync_write import GroupSyncWrite
    HAS_SDK = True
    HAS_SYNC_WRITE = True
except ImportError:
    try:
        from feetech_servo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        from feetech_servo_sdk.group_sync_write import GroupSyncWrite
        HAS_SDK = True
        HAS_SYNC_WRITE = True
    except ImportError:
        HAS_SDK = False
        HAS_SYNC_WRITE = False
        PortHandler = None
        PacketHandler = None
        COMM_SUCCESS = 0


def auto_detect_port():
    """
    Auto-detect serial port for servo adapter.

    Returns:
        str: Path to serial port, or None if not found

    Checks in order:
        1. Milk-V UART (/dev/ttyS2)
        2. macOS USB serial (/dev/tty.usbserial-*, /dev/tty.usbmodem-*)
        3. Linux USB serial (/dev/ttyUSB*, /dev/ttyACM*)
    """
    # Milk-V UART2 (direct connection)
    if os.path.exists("/dev/ttyS2"):
        return "/dev/ttyS2"

    # macOS USB serial (Waveshare adapter)
    patterns = [
        "/dev/tty.usbserial-*",
        "/dev/tty.usbmodem*",
    ]
    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]

    # Linux USB serial
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if ports:
        return ports[0]

    return None


def angle_to_steps(angle):
    """Convert angle (degrees) to servo steps."""
    return int(angle * STEPS_PER_REV / 360)


def steps_to_angle(steps):
    """Convert servo steps to angle (degrees)."""
    return steps * 360 / STEPS_PER_REV


class RawServoController:
    """
    Low-level serial controller for Milk-V Duo S (no SDK required).

    Uses raw serial protocol to communicate with Feetech servos.
    Required when running directly on Milk-V without Python SDK support.
    """

    def __init__(self, port="/dev/ttyS2", baudrate=DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.fd = None
        self.connected = False

    def connect(self):
        """Open serial connection."""
        try:
            os.system(f"stty -F {self.port} {self.baudrate} raw -echo cs8 -cstopb -parenb 2>/dev/null")
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self.connected = True
            return True
        except Exception as e:
            print(f"RawServoController: Failed to open {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection."""
        if self.fd:
            os.close(self.fd)
            self.fd = None
        self.connected = False

    def _checksum(self, data):
        """Calculate packet checksum."""
        return (~sum(data)) & 0xFF

    def _flush(self):
        """Flush any pending data from the buffer."""
        try:
            while True:
                data = os.read(self.fd, 256)
                if not data:
                    break
        except:
            pass

    def ping(self, servo_id):
        """
        Ping a servo to check if it's responding.

        Returns:
            tuple: (success: bool, error_byte: int or None)
        """
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
        """
        Read raw encoder position (0-65535 steps).

        Returns:
            int: Raw position, or None if read failed
        """
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
        """Write raw position to servo (0-65535 steps)."""
        position = int(position) & 0xFFFF
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        packet = bytes([0xFF, 0xFF, servo_id, 0x05, 0x03, ADDR_GOAL_POSITION, pos_l, pos_h])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.01)

    def set_speed(self, servo_id, speed):
        """Set goal speed for servo (steps/second). 0 means don't move."""
        speed = int(speed) & 0xFFFF
        speed_l = speed & 0xFF
        speed_h = (speed >> 8) & 0xFF
        packet = bytes([0xFF, 0xFF, servo_id, 0x05, 0x03, ADDR_GOAL_SPEED, speed_l, speed_h])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.01)


def _swap_bytes(value):
    """
    Swap bytes for 16-bit value to fix SDK byte-order issue.

    The scservo_sdk uses big-endian (low << 8 | high) but STS3215 servos
    use little-endian (low | high << 8). This swaps the byte order.
    """
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)


class SDKServoController:
    """
    SDK-based servo controller for PC (via USB adapter).

    Uses scservo_sdk or feetech_servo_sdk for communication.
    Supports sync write for efficient multi-servo commands.

    NOTE: The SDK has a byte-order mismatch with STS3215 servos.
    All 2-byte reads/writes are byte-swapped to correct this.
    """

    def __init__(self, port, baudrate=DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.connected = False

    def connect(self):
        """Open serial connection via SDK."""
        if not HAS_SDK:
            print("SDKServoController: SDK not available")
            return False

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(1)  # Protocol version 1

        if not self.port_handler.openPort():
            print(f"SDKServoController: Failed to open {self.port}")
            return False

        if not self.port_handler.setBaudRate(self.baudrate):
            print(f"SDKServoController: Failed to set baudrate {self.baudrate}")
            return False

        self.connected = True
        return True

    def disconnect(self):
        """Close serial connection."""
        if self.port_handler:
            self.port_handler.closePort()
        self.connected = False

    def ping(self, servo_id):
        """
        Ping a servo to check if it's responding.

        Returns:
            tuple: (success: bool, error_byte: int or None)
        """
        model, result, error = self.packet_handler.ping(self.port_handler, servo_id)
        return result == COMM_SUCCESS, error

    def read_position(self, servo_id):
        """
        Read raw encoder position (0-4095 steps for STS3215).

        Returns:
            int: Raw position, or None if read failed

        Note: Byte-swap applied to fix SDK endianness mismatch.
        """
        pos, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, ADDR_CURRENT_POSITION
        )
        if result != COMM_SUCCESS:
            return None
        # Swap bytes to fix SDK endianness
        pos = _swap_bytes(pos)
        # Sanity check for 12-bit encoder (0-4095)
        if pos > 4095:
            return None
        return pos

    def write_position(self, servo_id, position):
        """Write raw position to servo (0-4095 steps for STS3215)."""
        position = int(position) & 0x0FFF  # 12-bit range
        # Swap bytes before sending to fix SDK endianness
        position = _swap_bytes(position)
        self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, ADDR_GOAL_POSITION, position
        )

    def set_speed(self, servo_id, speed):
        """Set goal speed for servo (steps/second). 0 means don't move."""
        speed = int(speed) & 0xFFFF
        # Swap bytes before sending to fix SDK endianness
        speed = _swap_bytes(speed)
        self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, ADDR_GOAL_SPEED, speed
        )


class ServoController:
    """
    High-level servo controller with calibration, rate limiting, and sync write.

    This is the main interface for teleoperation. It wraps either SDKServoController
    or RawServoController depending on the environment.

    Features:
        - Automatic calibration from JSON file
        - First-frame zero offset capture
        - Rate limiting (max degrees per frame)
        - Dead zone (ignore small changes)
        - Sync write for multi-servo efficiency
        - Angle <-> steps conversion
    """

    # Rate limiting settings (disabled for now)
    MAX_ANGLE_CHANGE_PER_FRAME = 360.0  # Disabled - allow full range
    DEAD_ZONE_DEGREES = 0.0  # Disabled - respond to all changes

    def __init__(self, port, baudrate=DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.calibration = {}
        self.connected = False
        self.last_angles = {}  # Track last commanded angle per servo

        # Choose controller based on environment
        self._use_raw = (port == "/dev/ttyS2" or not HAS_SDK)
        self._ctrl = None

    def connect(self):
        """Open serial connection to servos."""
        if self._use_raw:
            self._ctrl = RawServoController(self.port, self.baudrate)
        else:
            self._ctrl = SDKServoController(self.port, self.baudrate)

        if not self._ctrl.connect():
            return False

        self.connected = True
        print(f"ServoController: Connected to {self.port} at {self.baudrate} bps")
        return True

    def disconnect(self):
        """Close serial connection."""
        if self._ctrl:
            self._ctrl.disconnect()
        self.connected = False

    def load_calibration(self, filepath):
        """
        Load calibration file with zero offsets.

        Args:
            filepath: Path to JSON calibration file

        Returns:
            bool: True if loaded successfully
        """
        if os.path.exists(filepath):
            with open(filepath, "r") as f:
                self.calibration = json.load(f)
            print(f"ServoController: Loaded calibration for servos {list(self.calibration.keys())}")
            return True
        print(f"ServoController: Calibration file not found: {filepath}")
        return False

    def save_calibration(self, filepath):
        """Save current calibration to file."""
        with open(filepath, "w") as f:
            json.dump(self.calibration, f, indent=2)
        print(f"ServoController: Saved calibration to {filepath}")

    def calibrate(self, servo_ids, names=None):
        """
        Calibrate servos by saving current positions as zero.

        Args:
            servo_ids: List of servo IDs to calibrate
            names: Optional dict mapping servo_id -> name
        """
        if names is None:
            names = {}

        for sid in servo_ids:
            pos = self.read_position_raw(sid)
            if pos is None:
                print(f"  ID {sid}: FAILED TO READ")
                continue

            self.calibration[str(sid)] = {
                "zero_offset": pos,
                "name": names.get(sid, f"servo_{sid}")
            }
            print(f"  ID {sid}: zero = {pos} (raw steps)")

    def ping(self, servo_id):
        """Ping a servo to check if it's responding."""
        if not self.connected:
            return False, None
        return self._ctrl.ping(servo_id)

    def read_position_raw(self, servo_id):
        """Read raw encoder position (0-65535 steps)."""
        if not self.connected:
            return None
        return self._ctrl.read_position(servo_id)

    def write_position_raw(self, servo_id, position):
        """Write raw position to servo (0-65535 steps)."""
        if not self.connected:
            return
        self._ctrl.write_position(servo_id, position)

    def set_speed(self, servo_id, speed):
        """Set goal speed for a servo (steps/second)."""
        if not self.connected:
            return
        self._ctrl.set_speed(servo_id, speed)

    def initialize_servos(self, servo_ids, speed=None):
        """
        Initialize servos with default speed for movement.

        IMPORTANT: STS3215 servos require non-zero speed to move.
        Speed=0 means "don't move", not "max speed".

        Args:
            servo_ids: List of servo IDs to initialize
            speed: Speed in steps/second (default: DEFAULT_SERVO_SPEED)
        """
        if not self.connected:
            return

        if speed is None:
            speed = DEFAULT_SERVO_SPEED

        print(f"ServoController: Initializing {len(servo_ids)} servos with speed={speed}")
        for sid in servo_ids:
            self._ctrl.set_speed(sid, speed)
            time.sleep(0.005)

    def read_angle(self, servo_id):
        """
        Read current angle from servo (relative to calibrated zero).

        Returns:
            float: Angle in degrees, or None if read failed
        """
        if not self.connected:
            return None

        key = str(servo_id)
        if key not in self.calibration:
            return None

        pos = self._ctrl.read_position(servo_id)
        if pos is None:
            return None

        zero_offset = self.calibration[key]["zero_offset"]
        return steps_to_angle(pos - zero_offset)

    def read_angles(self, servo_ids):
        """
        Read current angles from multiple servos.

        Returns:
            dict: {servo_id: angle} for successfully read servos
        """
        result = {}
        for servo_id in servo_ids:
            angle = self.read_angle(servo_id)
            if angle is not None:
                result[servo_id] = angle
        return result

    def write_angle(self, servo_id, angle, force=False):
        """
        Write angle to servo (relative to calibrated zero).

        Args:
            servo_id: Servo ID
            angle: Target angle in degrees
            force: If True, bypass rate limiting and dead zone

        Returns:
            bool: True if command was sent
        """
        if not self.connected:
            return False

        key = str(servo_id)
        if key not in self.calibration:
            return False

        # Apply dead zone and rate limiting (unless force=True)
        if not force:
            last = self.last_angles.get(servo_id)
            if last is not None:
                # Dead zone
                if abs(angle - last) < self.DEAD_ZONE_DEGREES:
                    return False
                # Rate limit
                delta = angle - last
                if abs(delta) > self.MAX_ANGLE_CHANGE_PER_FRAME:
                    angle = last + self.MAX_ANGLE_CHANGE_PER_FRAME * (1 if delta > 0 else -1)

        zero_offset = self.calibration[key]["zero_offset"]
        target_steps = zero_offset + angle_to_steps(angle)
        target_steps = max(0, min(4095, int(target_steps)))  # 12-bit encoder range

        self._ctrl.write_position(servo_id, target_steps)
        self.last_angles[servo_id] = angle
        time.sleep(0.001)
        return True

    def write_angles(self, angle_dict, force=False):
        """
        Write multiple angles at once (uses sync write if available).

        Args:
            angle_dict: {servo_id: angle} mapping
            force: If True, bypass rate limiting and dead zone
        """
        if not self.connected:
            return

        # Use individual writes (sync write disabled for now due to byte-order issues)
        for servo_id, angle in angle_dict.items():
            self.write_angle(servo_id, angle, force=force)
        time.sleep(0.002)

    def _sync_write_angles(self, angle_dict, force=False):
        """Internal: Use sync write for efficient multi-servo commands."""
        sync_write = GroupSyncWrite(
            self._ctrl.port_handler, self._ctrl.packet_handler,
            ADDR_GOAL_POSITION, 2  # 2 bytes for position
        )

        servos_added = 0
        angles_to_update = {}

        for servo_id, angle in angle_dict.items():
            key = str(servo_id)
            if key not in self.calibration:
                continue

            # Apply dead zone and rate limiting
            if not force:
                last = self.last_angles.get(servo_id)
                if last is not None:
                    if abs(angle - last) < self.DEAD_ZONE_DEGREES:
                        continue
                    delta = angle - last
                    if abs(delta) > self.MAX_ANGLE_CHANGE_PER_FRAME:
                        angle = last + self.MAX_ANGLE_CHANGE_PER_FRAME * (1 if delta > 0 else -1)

            zero_offset = self.calibration[key]["zero_offset"]
            target_steps = zero_offset + angle_to_steps(angle)
            target_steps = max(0, min(4095, int(target_steps)))  # 12-bit encoder range

            # addParam takes raw bytes in [low, high] order - no swap needed
            # (unlike write2ByteTxRx which needs swap due to SDK's internal byte order)
            sync_write.addParam(servo_id, [target_steps & 0xFF, (target_steps >> 8) & 0xFF])
            angles_to_update[servo_id] = angle
            servos_added += 1

        if servos_added > 0:
            sync_write.txPacket()
            for servo_id, angle in angles_to_update.items():
                self.last_angles[servo_id] = angle
            time.sleep(0.001 * servos_added + 0.002)

    def move_to_angle(self, servo_id, angle):
        """
        Move servo to absolute angle (relative to calibrated zero).
        Bypasses rate limiting for direct positioning.
        """
        return self.write_angle(servo_id, angle, force=True)

    def reset_to_neutral(self, servo_ids):
        """Reset specified servos to neutral (0 degrees)."""
        self.write_angles({sid: 0 for sid in servo_ids}, force=True)


def scan_servos(ctrl, id_range=range(1, 20)):
    """
    Scan for responding servos.

    Args:
        ctrl: ServoController instance (must be connected)
        id_range: Range of IDs to scan

    Returns:
        list: [(servo_id, position), ...] for found servos
    """
    found = []
    for sid in id_range:
        success, _ = ctrl.ping(sid)
        if success:
            pos = ctrl.read_position_raw(sid)
            found.append((sid, pos))
            print(f"  ID {sid}: position {pos}")
    return found
