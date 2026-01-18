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
import logging
import os
import time

# Configure module logger
logger = logging.getLogger(__name__)

def configure_logging(level=logging.INFO, log_file=None):
    """
    Configure logging for servo controller.

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR)
        log_file: Optional path to log file
    """
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # File handler (optional)
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    logger.setLevel(level)

# STS3215 constants
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_TIME = 44
ADDR_GOAL_SPEED = 46
ADDR_CURRENT_POSITION = 56
ADDR_CURRENT_SPEED = 58
ADDR_CURRENT_LOAD = 60
ADDR_CURRENT_VOLTAGE = 62  # Voltage in 0.1V units
ADDR_CURRENT_TEMPERATURE = 63  # Temperature in Celsius

# Error byte bit flags (from ping response)
ERROR_VOLTAGE = 0x01
ERROR_ANGLE_LIMIT = 0x02
ERROR_OVERHEAT = 0x04
ERROR_RANGE = 0x08
ERROR_CHECKSUM = 0x10
ERROR_OVERLOAD = 0x20
ERROR_INSTRUCTION = 0x40

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
            logger.error(f"RawServoController: Failed to open {self.port}: {e}")
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

    def read_temperature(self, servo_id):
        """
        Read servo temperature in Celsius.

        Returns:
            int: Temperature in Celsius, or None if read failed
        """
        self._flush()
        packet = bytes([0xFF, 0xFF, servo_id, 0x04, 0x02, ADDR_CURRENT_TEMPERATURE, 1])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.02)
        try:
            resp = os.read(self.fd, 64)
            if resp and len(resp) >= 7:
                return resp[5]
        except:
            pass
        return None

    def read_load(self, servo_id):
        """
        Read current load (torque) from servo.

        Returns:
            tuple: (load_value, direction) where direction is 1 for CW, 0 for CCW
                   load_value is 0-1023 (10-bit), or (None, None) if failed
        """
        self._flush()
        packet = bytes([0xFF, 0xFF, servo_id, 0x04, 0x02, ADDR_CURRENT_LOAD, 2])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.02)
        try:
            resp = os.read(self.fd, 64)
            if resp and len(resp) >= 8:
                load = resp[5] | (resp[6] << 8)
                direction = (load >> 10) & 0x01
                load_value = load & 0x03FF
                return load_value, direction
        except:
            pass
        return None, None

    def read_voltage(self, servo_id):
        """
        Read supply voltage in volts.

        Returns:
            float: Voltage in V, or None if read failed
        """
        self._flush()
        packet = bytes([0xFF, 0xFF, servo_id, 0x04, 0x02, ADDR_CURRENT_VOLTAGE, 1])
        packet += bytes([self._checksum(packet[2:])])
        os.write(self.fd, packet)
        time.sleep(0.02)
        try:
            resp = os.read(self.fd, 64)
            if resp and len(resp) >= 7:
                return resp[5] / 10.0  # Convert to volts
        except:
            pass
        return None


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
            logger.error("SDKServoController: SDK not available")
            return False

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(1)  # Protocol version 1

        if not self.port_handler.openPort():
            logger.error(f"SDKServoController: Failed to open {self.port}")
            return False

        if not self.port_handler.setBaudRate(self.baudrate):
            logger.error(f"SDKServoController: Failed to set baudrate {self.baudrate}")
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
        return pos

    def write_position(self, servo_id, position):
        """Write raw position to servo."""
        position = int(position) & 0xFFFF  # 16-bit range (no clipping)
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

    def read_temperature(self, servo_id):
        """
        Read servo temperature in Celsius.

        Returns:
            int: Temperature in Celsius, or None if read failed
        """
        temp, result, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, servo_id, ADDR_CURRENT_TEMPERATURE
        )
        if result != COMM_SUCCESS:
            return None
        return temp

    def read_load(self, servo_id):
        """
        Read current load (torque) from servo.

        Returns:
            tuple: (load_value, direction) where direction is 1 for CW, 0 for CCW
                   load_value is 0-1023 (10-bit), or (None, None) if failed
        """
        load, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, ADDR_CURRENT_LOAD
        )
        if result != COMM_SUCCESS:
            return None, None
        # Swap bytes to fix SDK endianness
        load = _swap_bytes(load)
        direction = (load >> 10) & 0x01
        load_value = load & 0x03FF
        return load_value, direction

    def read_voltage(self, servo_id):
        """
        Read supply voltage in volts.

        Returns:
            float: Voltage in V, or None if read failed
        """
        voltage, result, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, servo_id, ADDR_CURRENT_VOLTAGE
        )
        if result != COMM_SUCCESS:
            return None
        return voltage / 10.0  # Convert to volts


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

    # Rate limiting settings for smooth, safe motion
    # MAX_ANGLE_CHANGE_PER_FRAME: Limits degrees per frame to prevent jerky motion
    #   At 30 FPS, 30°/frame = 900°/sec max velocity
    # DEAD_ZONE_DEGREES: Ignore small changes to reduce jitter from sensor noise
    MAX_ANGLE_CHANGE_PER_FRAME = 30.0  # Prevents violent jumps from gimbal lock
    DEAD_ZONE_DEGREES = 2.0  # Filter out sensor noise

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
        logger.info(f"Connected to {self.port} at {self.baudrate} bps")
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
            logger.info(f"Loaded calibration for servos {list(self.calibration.keys())}")
            return True
        logger.warning(f"Calibration file not found: {filepath}")
        return False

    def save_calibration(self, filepath):
        """Save current calibration to file."""
        with open(filepath, "w") as f:
            json.dump(self.calibration, f, indent=2)
        logger.info(f"Saved calibration to {filepath}")

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
                logger.error(f"ID {sid}: FAILED TO READ")
                continue

            self.calibration[str(sid)] = {
                "zero_offset": pos,
                "name": names.get(sid, f"servo_{sid}")
            }
            logger.debug(f"ID {sid}: zero = {pos} (raw steps)")

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

    def read_temperature(self, servo_id):
        """Read servo temperature in Celsius."""
        if not self.connected:
            return None
        return self._ctrl.read_temperature(servo_id)

    def read_load(self, servo_id):
        """
        Read current load (torque) from servo.

        Returns:
            tuple: (load_value, direction) or (None, None) if failed
        """
        if not self.connected:
            return None, None
        return self._ctrl.read_load(servo_id)

    def read_voltage(self, servo_id):
        """Read supply voltage in volts."""
        if not self.connected:
            return None
        return self._ctrl.read_voltage(servo_id)

    def get_servo_status(self, servo_id):
        """
        Get comprehensive status for a servo including diagnostics.

        Returns:
            dict: {
                'online': bool,
                'error': int or None (error byte from ping),
                'errors': list of error strings,
                'position': int (raw steps),
                'temperature': int (Celsius),
                'voltage': float (V),
                'load': int (0-1023),
                'load_direction': int (0=CCW, 1=CW)
            }
        """
        status = {
            'online': False,
            'error': None,
            'errors': [],
            'position': None,
            'temperature': None,
            'voltage': None,
            'load': None,
            'load_direction': None,
        }

        if not self.connected:
            return status

        # Ping to check online status and get error byte
        online, error = self._ctrl.ping(servo_id)
        status['online'] = online
        status['error'] = error

        if not online:
            return status

        # Decode error flags
        if error:
            if error & ERROR_VOLTAGE:
                status['errors'].append('voltage')
            if error & ERROR_ANGLE_LIMIT:
                status['errors'].append('angle_limit')
            if error & ERROR_OVERHEAT:
                status['errors'].append('overheat')
            if error & ERROR_RANGE:
                status['errors'].append('range')
            if error & ERROR_CHECKSUM:
                status['errors'].append('checksum')
            if error & ERROR_OVERLOAD:
                status['errors'].append('overload')
            if error & ERROR_INSTRUCTION:
                status['errors'].append('instruction')

        # Read position
        status['position'] = self._ctrl.read_position(servo_id)

        # Read temperature
        status['temperature'] = self._ctrl.read_temperature(servo_id)

        # Read voltage
        status['voltage'] = self._ctrl.read_voltage(servo_id)

        # Read load
        load, direction = self._ctrl.read_load(servo_id)
        status['load'] = load
        status['load_direction'] = direction

        return status

    def check_servo_health(self, servo_id, temp_warn=60, temp_crit=70, load_warn=800):
        """
        Check servo health and return warnings.

        Args:
            servo_id: Servo ID to check
            temp_warn: Temperature warning threshold (Celsius)
            temp_crit: Temperature critical threshold (Celsius)
            load_warn: Load warning threshold (0-1023)

        Returns:
            list: Warning messages (empty if healthy)
        """
        warnings = []
        status = self.get_servo_status(servo_id)

        if not status['online']:
            warnings.append(f"Servo {servo_id}: OFFLINE")
            return warnings

        if status['errors']:
            warnings.append(f"Servo {servo_id}: ERRORS: {', '.join(status['errors'])}")

        if status['temperature'] is not None:
            if status['temperature'] >= temp_crit:
                warnings.append(f"Servo {servo_id}: CRITICAL TEMP {status['temperature']}°C")
            elif status['temperature'] >= temp_warn:
                warnings.append(f"Servo {servo_id}: HIGH TEMP {status['temperature']}°C")

        if status['load'] is not None and status['load'] >= load_warn:
            warnings.append(f"Servo {servo_id}: HIGH LOAD {status['load']}/1023")

        return warnings

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

        logger.info(f"Initializing {len(servo_ids)} servos with speed={speed}")
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
        target_steps = int(target_steps) & 0xFFFF  # 16-bit wrap (no clipping)

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

        # Use sync write for SDK controller (4x faster than individual writes)
        # Key insight: addParam() passes raw bytes directly to wire - NO swap needed
        if HAS_SYNC_WRITE and not self._use_raw:
            self._sync_write_angles(angle_dict, force=force)
        else:
            # Fall back to individual writes for raw controller
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
            target_steps = int(target_steps) & 0xFFFF  # 16-bit wrap (no clipping)

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
            logger.debug(f"Found ID {sid}: position {pos}")
    return found
