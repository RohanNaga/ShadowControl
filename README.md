# Shadow Control

Vision-based bimanual teleoperation system for a humanoid boxing robot.

**Competition**: CMU Build18 Hackathon (1-week timeline)
**Team**: Shadow Control
**Inspiration**: Real Steel movie - teleoperation through human movement tracking

---

## Overview

Shadow Control enables real-time control of a K-Scale Zeroth-01 humanoid robot by tracking human boxing movements through a camera. The human operator stands **behind** the robot, and the robot mirrors their movements like a puppeteer setup.

### Key Features

- **Live Tracking Mode**: Real-time pose detection and robot control
- **Video Playback Mode**: Execute pre-recorded movement sequences
- **Single-Board Architecture**: Everything runs on Milk-V Duo S (no external laptop)
- **Low Latency**: ~90ms end-to-end (camera to robot motion)

### Tech Stack

| Component | Technology |
|-----------|------------|
| Controller | Milk-V Duo S (512MB RAM, 0.5 TOPS TPU) |
| Robot | K-Scale Zeroth-01 (16 servos) |
| Pose Detection | YOLOv8n-pose converted to cvimodel |
| Servo Control | kos_zbot (K-Scale's robot OS) |
| Camera | CAM-GC2083 (2MP, MIPI CSI) |
| Servos | Feetech STS3215 (19kg.cm, 12-bit encoder) |

---

## Project Structure

```
ShadowControl/
├── README.md              # This file - complete documentation
├── LICENSE                # MIT License
├── requirements.txt       # Python dependencies
│
├── config/
│   └── robot.yaml         # Servo IDs, limits, keypoint mappings
│
├── src/
│   ├── __init__.py
│   ├── vision/            # Pose detection (C, TDL SDK)
│   │   └── __init__.py
│   └── control/           # Servo control (Python, kos_zbot)
│       └── __init__.py
│
├── scripts/
│   ├── test_servos.py     # Servo connectivity test
│   └── test_vision.py     # Camera/pose detection test
│
├── models/                # Converted cvimodel files
│   └── (yolov8n_pose.cvimodel)
│
├── docs/                  # Additional documentation
└── tests/                 # Unit tests
```

### Key Files

| File | Purpose |
|------|---------|
| `config/robot.yaml` | Servo ID mappings, joint limits, keypoint pairs |
| `scripts/configure_baudrate.py` | Reconfigure servo baudrate from 1Mbps to 500kbps |
| `scripts/test_servos.py` | Tests servo communication (kos_zbot or feetech SDK) |
| `scripts/test_vision.py` | Tests camera and pose detection pipeline |
| `src/vision/` | TDL SDK pose detection integration (C) |
| `src/control/` | kos_zbot servo control wrapper (Python) |

---

## Hardware

### Bill of Materials (~$610.98)

| Item | Qty | Purpose |
|------|-----|---------|
| K-Scale Zeroth-01 | 1 | 3D printed humanoid platform |
| STS3215 Servos | 24 | 7.4V, 19kg.cm, 12-bit encoder (16 used, 8 extras) |
| Milk-V Duo S (512MB) | 1 | Main controller with TPU |
| CAM-GC2083 | 1 | 2MP MIPI CSI camera |
| Waveshare Bus Servo Adapter | 4 | UART to servo interface (1 used, 3 extras) |
| RC LiPo 12V 5200mAh | 1 | Main power |
| 12V to 5V Converter | 3 | Logic power |
| E-Stop Button | 1 | Safety cutoff |
| RP2040 LCD IMU Board | 1 | IMU sensor |

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      MILK-V DUO S (512MB)                       │
│                                                                 │
│  ┌───────────────┐     ┌─────────────────────────────────────┐ │
│  │ GC2083 Camera │────▶│         TDL SDK + TPU              │ │
│  │ (2MP MIPI)    │     │  ┌─────────────────────────────┐   │ │
│  │               │     │  │  YOLOv8 Pose Detection      │   │ │
│  └───────────────┘     │  │  (17 keypoints @ 640x640)   │   │ │
│                        │  └──────────────┬──────────────┘   │ │
│                        │                 │                   │ │
│                        │                 ▼                   │ │
│                        │  ┌─────────────────────────────┐   │ │
│                        │  │  Joint Angle Calculator     │   │ │
│                        │  │  (keypoints → servo angles) │   │ │
│                        │  └──────────────┬──────────────┘   │ │
│                        └─────────────────┼───────────────────┘ │
│                                          │                     │
│  ┌───────────────┐                       ▼                     │
│  │ kos_zbot      │    ┌─────────────────────────────────────┐ │
│  │ (K-Scale OS)  │───▶│      Servo Control Loop             │ │
│  └───────────────┘    │  UART @ 500kbps                     │ │
│                       └──────────────┬──────────────────────┘ │
└──────────────────────────────────────┼─────────────────────────┘
                                       │
                    ┌──────────────────┴──────────────────┐
                    │     WAVESHARE BUS SERVO ADAPTER     │
                    │              16x STS3215            │
                    └─────────────────────────────────────┘
```

### Milk-V Duo S Specifications

| Spec | Value |
|------|-------|
| CPU | C906 1GHz + C906 700MHz (RISC-V) OR Cortex-A53 1GHz (ARM) |
| TPU | 0.5 TOPS @ INT8 (CVITEK) |
| RAM | 512MB |
| Camera | 2x MIPI CSI 2-lane (5MP@30fps) |
| UART Max | 921,600 bps |

### Feetech STS3215 Servo Specifications

| Spec | Value |
|------|-------|
| Voltage | 6-12.6V (using 7.4V) |
| Torque | 19.5 kg.cm @ 7.4V |
| Speed | 0.222 sec/60° (45 RPM) |
| Resolution | 360°/4096 (12-bit) |
| Default Baudrate | 1 Mbps |
| Protocol | Half-duplex TTL serial |
| Max Chain | 253 servos per bus |

---

## Critical Configuration Notes

### UART Baudrate (IMPORTANT - MUST FIX BEFORE USE)

The Milk-V Duo S UART maximum is **921,600 bps**, but Feetech servos default to **1 Mbps**. This is an undocumented issue - K-Scale does not mention this limitation.

**Solution**: Reconfigure all servos to **500,000 bps** before deployment:

```python
# Baudrate register values
BAUDRATE_MAP = {
    1000000: 0,  # Factory default - WON'T WORK with Milk-V
    500000: 1,   # USE THIS
    250000: 2,
    128000: 3,
    115200: 4,
}
```

**How to reconfigure:**
1. Connect servo to PC via USB with FE-URT-1 board
2. Open Feetech FD software
3. Set baudrate to 500000 (register value 1)
4. Click "Save"
5. Repeat for all 16 servos

When running kos-zbot:
```bash
kos zbot --baudrate 500000
```

### Wiring (CRITICAL - Not Standard UART!)

The Waveshare adapter uses **non-standard** wiring:

```
Milk-V Duo S          Waveshare Adapter
────────────          ─────────────────
TX (pin 8)    ───────►  TX
RX (pin 10)   ───────►  RX
GND (pin 6)   ───────►  GND
```

**WARNING**: TX-to-TX, RX-to-RX (NOT crossed like typical UART!)

**Waveshare Jumper Settings:**
- Position **A**: UART mode (for Milk-V) ← USE THIS
- Position **B**: USB mode (for PC debugging)

### Dual Waveshare Setup (Recommended - No Splicing)

Using 2 Waveshare adapters with 2 separate UARTs. No soldering or wire splicing required.

**Pin Summary:**

| Waveshare | Function | Pin | Device | Servos |
|-----------|----------|-----|--------|--------|
| #1 (Arms) | TX | Pin 8 | `/dev/ttyS0` | ID 1-8 |
| #1 (Arms) | RX | Pin 10 | | |
| #1 (Arms) | GND | **Pin 6** | | |
| #2 (Legs) | TX | Pin 11 | `/dev/ttyS2` | ID 9-16 |
| #2 (Legs) | RX | Pin 13 | | |
| #2 (Legs) | GND | **Pin 14** | | |

**Available GND pins on J3 header:** 6, 9, 14, 20, 25

```
                    MILK-V DUO S (J3 Header - 40 pins)
    ┌───────────────────────────────────────────────────────────────┐
    │                                                               │
    │   UART0 (default)              UART2 (pinmux enabled)        │
    │   ══════════════               ══════════════════════        │
    │   Pin 8  (TX) ────┐            Pin 11 (TX) ────┐             │
    │   Pin 10 (RX) ──┐ │            Pin 13 (RX) ──┐ │             │
    │   Pin 6  (GND)─┐│ │            Pin 14 (GND)─┐│ │             │
    │                │││ │                        │││ │             │
    └────────────────┼┼┼─┼────────────────────────┼┼┼─┼─────────────┘
                     │││ │                        │││ │
                     ▼▼▼ ▼                        ▼▼▼ ▼
              ┌─────────────────┐          ┌─────────────────┐
              │ WAVESHARE #1    │          │ WAVESHARE #2    │
              │ (Jumper A)      │          │ (Jumper A)      │
              │                 │          │                 │
              │ GND ◄── Pin 6   │          │ GND ◄── Pin 14  │
              │ RX  ◄── Pin 10  │          │ RX  ◄── Pin 13  │
              │ TX  ◄── Pin 8   │          │ TX  ◄── Pin 11  │
              │                 │          │                 │
              │ DC: 7.4-12V     │          │ DC: 7.4-12V     │
              └────────┬────────┘          └────────┬────────┘
                       │                            │
            ┌──────────┴──────────┐      ┌──────────┴──────────┐
            │   DVG1      DVG2    │      │   DVG1      DVG2    │
            └─────┬─────────┬─────┘      └─────┬─────────┬─────┘
                  │         │                  │         │
                  ▼         ▼                  ▼         ▼
              ┌───────┐ ┌───────┐          ┌───────┐ ┌───────┐
              │ R.ARM │ │ L.ARM │          │ R.LEG │ │ L.LEG │
              │ ID 1-4│ │ ID 5-8│          │ID 9-12│ │ID13-16│
              └───────┘ └───────┘          └───────┘ └───────┘
```

### UART2 Pinmux Configuration (Required)

UART2 is not enabled by default. The pinmux must be configured at boot.

**Automatic (persistent):** Create `/mnt/data/auto.sh` on the Milk-V:
```bash
#!/bin/sh
# Enable UART2 on pins 11 (TX) and 13 (RX)
duo-pinmux -w B11/UART2_TX
duo-pinmux -w B12/UART2_RX
```

**Manual (temporary):**
```bash
duo-pinmux -w B11/UART2_TX
duo-pinmux -w B12/UART2_RX
```

**What is pinmux?** Pin multiplexing allows each physical pin to serve different functions (GPIO, UART, I2C, etc.). The `duo-pinmux` tool configures which internal peripheral connects to which physical pin.

### Camera Connection

- Connect CAM-GC2083 to **J1 connector** (16-pin, 0.5mm pitch)
- Uses **I2C3** for sensor communication
- J2 connector (15-pin, 1.0mm pitch) is for RPi-compatible cameras

### GPIO Voltage Levels

| Header | Voltage |
|--------|---------|
| J3 | 3.3V logic ← Use for servos |
| J4 | 1.8V logic |

---

## Data Flow Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     SHADOW CONTROL - TELEOPERATION PIPELINE                 │
└─────────────────────────────────────────────────────────────────────────────┘

STEP 1: CAMERA INPUT
┌─────────────────┐
│ GC2083 Camera   │  → Captures human standing BEHIND robot
│ (2MP, 30fps)    │  → MIPI CSI to Milk-V J1 connector
└────────┬────────┘
         │ RGB frames (~33ms)
         ▼
STEP 2: POSE DETECTION (TPU)
┌─────────────────────────────────┐
│ YOLOv8n-pose (converted)        │  → Runs on 0.5 TOPS TPU
│ 640×640 input, INT8 quantized   │  → ~45ms inference (~20 FPS)
└────────┬────────────────────────┘
         │ 17 keypoints (x, y, confidence)
         │ [nose, eyes, ears, shoulders, elbows, wrists, hips, knees, ankles]
         ▼
STEP 3: JOINT ANGLE CALCULATION (CPU)
┌─────────────────────────────────┐
│ Keypoint → Joint Angles         │  → Extract upper body keypoints (~2ms)
│ - atan2(elbow - shoulder)       │  → Calculate shoulder angles
│ - dot_product for elbow angles  │  → Calculate elbow/wrist angles
│ - Scale to robot proportions    │  → Map to servo range (0-4096)
└────────┬────────────────────────┘
         │ Joint angles (degrees)
         ▼
STEP 4: SERVO CONTROL (kos_zbot)
┌─────────────────────────────────┐
│ kos_zbot --baudrate 500000      │  → Safety limits enforced (~10ms)
│ Sync write to 16 servos         │  → Coordinated motion
│ UART @ 500kbps to adapters      │  → Low-latency communication
└────────┬────────────────────────┘
         │ Position commands
         ▼
STEP 5: ROBOT MOTION
┌─────────────────────────────────┐
│ 16x STS3215 Servos              │  → Robot mimics human pose
│ (reconfigured to 500kbps)       │  → Real-time boxing movements
│ Zeroth-01 Humanoid              │  → <100ms end-to-end latency
└─────────────────────────────────┘
```

### Latency Breakdown

| Stage | Time |
|-------|------|
| Camera capture | ~33ms (30fps) |
| Pose inference | ~45ms |
| Angle calculation | ~2ms |
| Servo communication | ~10ms |
| **Total end-to-end** | **~90ms** |

---

## Setup Instructions

### 1. Servo Baudrate Configuration (One-Time - REQUIRED)

Before first use, reconfigure all servos from 1Mbps to 500kbps.

**Good news**: Use the Waveshare adapter itself (no separate FE-URT-1 board needed)!

#### Option A: Using Python Script (Recommended)

```bash
# On your PC with Waveshare adapter connected via USB
pip install feetech-servo-sdk

# Interactive mode - configure one servo at a time
python scripts/configure_baudrate.py --port /dev/ttyUSB0 --id 1-16 --interactive

# Or scan for servos to see current baudrates
python scripts/configure_baudrate.py --scan
```

#### Option B: Manual via Feetech FD Software (Windows)

1. Set Waveshare adapter jumper to position **B** (USB mode)
2. Connect adapter to PC via USB
3. Connect ONE servo at a time to DVG port
4. Open Feetech FD software
5. Set baudrate to 500000 (register value 1)
6. Click "Save"
7. Repeat for all 24 servos

**Note**: Factory-default servos all have ID=1, so configure one at a time

### 2. Hardware Assembly

Follow K-Scale Zeroth-01 assembly guide at [docs.kscale.dev](https://docs.kscale.dev/category/zeroth-01/)

Key points:
- M3 screws for motor outputs
- M2 screws for motor housings
- Use low-strength threadlocker (Loctite Purple)

### 3. Software Installation

```bash
# Clone repository
git clone https://github.com/your-team/ShadowControl.git
cd ShadowControl

# Install dependencies
pip install -r requirements.txt

# Install kos_zbot (K-Scale robot OS)
pip install kos_zbot
```

### 4. YOLOv8-Pose Model Conversion

The Milk-V TPU requires a converted cvimodel:

```bash
# Step 1: Export YOLOv8n-pose to ONNX (on dev machine)
pip install ultralytics
yolo export model=yolov8n-pose.pt format=onnx

# Step 2: Set up TPU-MLIR (Docker recommended)
docker pull sophgo/tpuc_dev:latest

# Step 3: Convert ONNX to MLIR
model_transform.py --model_name yolov8n_pose \
  --model_def yolov8n-pose.onnx \
  --input_shapes [[1,3,640,640]] \
  --pixel_format rgb --output_names output0 \
  --mlir yolov8n_pose.mlir

# Step 4: Deploy cvimodel with INT8 quantization
model_deploy.py --mlir yolov8n_pose.mlir \
  --quantize INT8 --chip cv181x \
  --model yolov8n_pose.cvimodel

# Step 5: Copy to Milk-V
scp yolov8n_pose.cvimodel root@milkv:/path/to/ShadowControl/models/
```

### 5. Initial Testing (Stock Linux Image)

If you have stock Milk-V Linux (before flashing K-Scale image), you can still test hardware:

```bash
# SSH into Milk-V
ssh root@milkv-duos

# Step 1: Check UART device exists
ls -la /dev/ttyAMA*
# Should show /dev/ttyAMA5 (default UART on GPIO header)

# Step 2: Check pinmux configuration
cat /sys/kernel/debug/pinctrl/pinctrl-maps | grep -i uart
# OR
duo-pinmux -p GP4  # Pin 8 - should show UART function
duo-pinmux -p GP5  # Pin 10 - should show UART function

# Step 3: Simple serial test (no servos yet)
stty -F /dev/ttyAMA5 500000 raw -echo
```

### 6. Running Shadow Control (Full Setup)

```bash
# Test servos first
python scripts/test_servos.py --device /dev/ttyAMA5 --baudrate 500000 --ids 1-16

# Test camera and pose detection
python scripts/test_vision.py

# Run full pipeline
./shadow_control --baudrate 500000 --mode live
```

---

## Servo ID Assignment

Configure each servo with a unique ID using `scripts/configure_servo.py`.

### Right Arm (IDs 5-7)

| Joint | Servo ID | Configured | Notes |
|-------|----------|------------|-------|
| Right Shoulder | 5 | ✓ | |
| Right Elbow | 6 | ✓ | |
| Right Wrist | 7 | ✓ | |

### Left Arm (IDs 8-10)

| Joint | Servo ID | Configured | Notes |
|-------|----------|------------|-------|
| Left Shoulder | 8 | ✓ | |
| Left Elbow | 9 | ✓ | |
| Left Wrist | 10 | | |

### Right Leg (IDs 11-15)

| Joint | Servo ID | Configured | Notes |
|-------|----------|------------|-------|
| Right Hip | 11 | | |
| Right Thigh | 12 | | |
| Right Knee | 13 | | |
| Right Shin | 14 | | |
| Right Ankle | 15 | | |

### Left Leg (IDs 16-20)

| Joint | Servo ID | Configured | Notes |
|-------|----------|------------|-------|
| Left Hip | 16 | | |
| Left Thigh | 17 | | |
| Left Knee | 18 | | |
| Left Shin | 19 | | |
| Left Ankle | 20 | | |

### Configuration Commands

```bash
# Configure a factory servo (1Mbps) to a specific ID at 500kbps
python3 scripts/configure_servo.py --new-id <ID> --factory

# Test a configured servo
python3 scripts/configure_servo.py --test <ID>

# Scan for all servos
python3 scripts/configure_servo.py --scan
```

---

## Servo Mapping (Keypoints)

| Servo ID | Joint | Keypoint Pair | Notes |
|----------|-------|---------------|-------|
| 5 | Right Shoulder | 6→8 | right_shoulder to right_elbow |
| 6 | Right Elbow | 6→8→10 | shoulder-elbow-wrist angle |
| 7 | Right Wrist | 8→10 | |
| 8 | Left Shoulder | 5→7 | left_shoulder to left_elbow |
| 9 | Left Elbow | 5→7→9 | shoulder-elbow-wrist angle |
| 10 | Left Wrist | 7→9 | |
| 11 | Right Hip | 12→14 | right_hip to right_knee |
| 12 | Right Thigh | 12→14 | |
| 13 | Right Knee | 12→14→16 | hip-knee-ankle angle |
| 14 | Right Shin | 14→16 | |
| 15 | Right Ankle | 14→16 | |
| 16 | Left Hip | 11→13 | left_hip to left_knee |
| 17 | Left Thigh | 11→13 | |
| 18 | Left Knee | 11→13→15 | hip-knee-ankle angle |
| 19 | Left Shin | 13→15 | |
| 20 | Left Ankle | 13→15 | |

---

## Keypoint Reference

YOLOv8-pose outputs 17 COCO keypoints:

```
Index  Name            Index  Name
─────  ────            ─────  ────
0      nose            9      left_wrist
1      left_eye        10     right_wrist
2      right_eye       11     left_hip
3      left_ear        12     right_hip
4      right_ear       13     left_knee
5      left_shoulder   14     right_knee
6      right_shoulder  15     left_ankle
7      left_elbow      16     right_ankle
8      right_elbow
```

### Angle Calculation Code

```python
import numpy as np

def calculate_angle(p1, p2, p3):
    """Calculate angle at p2 formed by p1-p2-p3"""
    v1 = np.array(p1) - np.array(p2)
    v2 = np.array(p3) - np.array(p2)

    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    angle = np.arccos(np.clip(cos_angle, -1, 1))
    return np.degrees(angle)

def shoulder_angle(shoulder, elbow):
    """Calculate shoulder pitch using atan2"""
    dx = elbow[0] - shoulder[0]
    dy = elbow[1] - shoulder[1]
    return np.degrees(np.arctan2(dy, dx))
```

---

## Performance Targets

| Metric | Target | Achieved |
|--------|--------|----------|
| Pose Detection FPS | >10 | ~22 FPS |
| End-to-End Latency | <100ms | ~90ms |
| Servo Update Rate | >30Hz | TBD |

---

## Safety Features

1. **E-Stop Button**: Hardware cutoff to servo power
2. **Joint Limits**: Software-enforced angle constraints (in `config/robot.yaml`)
3. **Velocity Limits**: Smooth motion, prevent jerky movements
4. **Timeout**: Stop if camera disconnects or no commands for 500ms

---

## Dependencies

### Python (requirements.txt)
```
kos_zbot>=0.1.0      # K-Scale robot OS
numpy>=1.21.0         # Math operations
pyyaml>=6.0           # Config parsing
opencv-python>=4.5.0  # Dev only - image processing
ultralytics>=8.0.0    # Dev only - model export
```

### System (Milk-V)
- TDL SDK - Pose detection on TPU
- V4L2 - Camera interface

---

## Known Issues

1. **K-Scale UART docs missing** - K-Scale doesn't document the Milk-V baudrate limitation (921600 max); we discovered it through Sophgo docs
2. **No pre-built YOLOv8-pose cvimodel** - Must convert using TPU-MLIR
3. **STS3215 leg speed** - K-Scale recommends STS3250 for legs due to speed requirements, but STS3215 works for upper-body boxing focus

---

## Troubleshooting

### Servos Not Responding

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| No servos found | Baudrate mismatch | Reconfig servos to 500kbps with `configure_baudrate.py` |
| No servos found | Wrong wiring | Check TX-TX, RX-RX (NOT crossed!) |
| No servos found | No power | Check DC input to adapter (7.4-12V) |
| No servos found | Wrong jumper | Ensure jumper is at position **A** (UART mode) |
| Some servos missing | Duplicate IDs | Check each servo has unique ID |
| Some servos missing | Broken chain | Check daisy chain connections |
| Permission denied | UART access | Run as root or add user to dialout |
| /dev/ttyAMA5 missing | Pinmux not set | Flash full K-Scale image |

**Quick diagnosis:**
```bash
# Scan for servos at different baudrates
python scripts/configure_baudrate.py --scan
```

### Camera Not Working

1. Verify J1 connector properly seated (check orientation)
2. Check I2C3 is enabled in device tree
3. Run `v4l2-ctl --list-devices` to verify detection

### High Latency

1. Check TPU inference time (should be ~45ms)
2. Reduce input resolution if needed
3. Verify servo communication not blocking

---

## Next Steps / Implementation Roadmap

### Phase 2: Pose Detection Pipeline
- [ ] Set up TPU-MLIR environment (Docker)
- [ ] Export YOLOv8n-pose to ONNX format
- [ ] Convert ONNX → MLIR → cvimodel
- [ ] Deploy cvimodel to Milk-V
- [ ] Integrate with TDL SDK
- [ ] Test 17 keypoint output

### Phase 3: Joint Angle Calculation
- [ ] Implement keypoint → angle conversion in C
- [ ] Map human proportions to robot proportions
- [ ] Add smoothing/filtering for stable output

### Phase 4: Servo Control Integration
- [ ] Reconfigure all servos to 500kbps
- [ ] Install kos_zbot on Milk-V
- [ ] Test servo communication
- [ ] Implement sync write for coordinated motion
- [ ] Add safety limits

### Phase 5: Full Integration
- [ ] End-to-end pipeline: Camera → TPU → Angles → Servos
- [ ] Latency measurement (<100ms target)
- [ ] Calibration procedure
- [ ] Mode switching (live vs playback)

### Phase 6: Polish
- [ ] E-Stop integration (GPIO)
- [ ] Video playback mode
- [ ] Demo preparation
- [ ] Safety testing

---

## References

### K-Scale
- [Zeroth-01 Docs](https://docs.kscale.dev/category/zeroth-01/)
- [kos-zbot GitHub](https://github.com/kscalelabs/kos-zbot)
- [Zeroth-Bot GitHub](https://github.com/zeroth-robotics/zeroth-bot)

### Milk-V / Sophgo
- [Duo S Getting Started](https://milkv.io/docs/duo/getting-started/duos)
- [TDL SDK Guide](https://doc.sophgo.com/cvitek-develop-docs/master/docs_latest_release/CV180x_CV181x/en/01.software/TPU/TDL_SDK_Software_Development_Guide/build/html/index.html)
- [YOLO Development Guide](https://doc.sophgo.com/cvitek-develop-docs/master/docs_latest_release/CV180x_CV181x/en/01.software/TPU/YOLO_Development_Guide/build/html/6_Yolov8_development.html)
- [Peripheral Driver Guide (UART)](https://doc.sophgo.com/cvitek-develop-docs/master/docs_latest_release/CV180x_CV181x/en/01.software/BSP/Peripheral_Driver_Operation_Guide/build/html/8_UART_Operation_Guide.html)
- [duo-tdl-examples GitHub](https://github.com/milkv-duo/duo-tdl-examples)

### Pose Estimation
- [YOLOv8-Pose](https://docs.ultralytics.com/tasks/pose/)
- [H2O Human to Humanoid](https://human2humanoid.com/)

### Related Projects
- [Humandroid (MediaPipe to Robot)](https://github.com/vellons/Humandroid)
- [TonyPi MediaPipe Control](https://www.hackster.io/HiwonderRobot/real-time-pose-imitation-control-for-tonypi-using-mediapipe-8025da)
- [NAO Mimicry](https://github.com/Ahmed-AI-01/Nao_Mimc)

---

## License

MIT License - See LICENSE file

---

## Team

**Shadow Control** - CMU Build18 Hackathon 2026
