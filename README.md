# 6-DOF Robotic Arm — Arduino Controller

A fully articulated 6-degree-of-freedom robotic arm designed from scratch in SolidWorks and driven by an Arduino-based servo control system. The mechanical design mirrors real industrial robotics architectures — parallel linkages, modular joints, and a kinematic chain built for precision.

![Status](https://img.shields.io/badge/status-active_development-D4860A)
![Platform](https://img.shields.io/badge/platform-Arduino_Mega_2560-00979D)
![Language](https://img.shields.io/badge/language-C++-00599C)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Overview

Six degrees of freedom — the same mobility as a human arm. This project spans the full engineering pipeline: mechanical design in SolidWorks, kinematic modeling, and real-time servo control via Arduino. The arm features a parallel-linkage shoulder, articulated elbow, 2-axis wrist, and a mechanical gripper, all coordinated through a custom motion interpolation engine.

### Joint Architecture

```
                        ┌─── J3: Wrist Pitch
                        │    ┌─── J4: Wrist Roll
                        │    │    ┌─── J5: Gripper
    ┌───────────────────┤    │    │
    │   Upper Arm       └────┴────┴──── 🦾 End Effector
    │
    ├─── J2: Elbow
    │
    │   Lower Arm
    │   (Parallel Linkage)
    │
    ├─── J1: Shoulder
    │
    └─── J0: Base Rotation
         ═══════════════
         Circular Platform
```

| Joint | Function | Servo | Range | Home | Notes |
|-------|----------|-------|-------|------|-------|
| J0 | Base Rotation | MG996R | 0°–180° | 90° | Horizontal yaw on circular platform |
| J1 | Shoulder | MG996R | 15°–165° | 90° | Parallel-linkage lower arm; limits prevent base collision |
| J2 | Elbow | MG996R | 0°–180° | 90° | Upper arm articulation |
| J3 | Wrist Pitch | MG90S | 0°–180° | 90° | End-effector tilt |
| J4 | Wrist Roll | MG90S | 0°–180° | 90° | End-effector rotation |
| J5 | Gripper | MG90S | 10°–73° | 10° | Claw open (10°) / closed (73°) |

---

## Features

- **Coordinated Joint Motion** — All 6 joints interpolate proportionally toward their targets simultaneously, producing smooth, natural arm trajectories instead of jerky sequential moves
- **Mechanical Joint Limits** — Software-enforced angle constraints match the physical range of each joint in the SolidWorks assembly, preventing self-collision and servo damage
- **Serial Command Interface** — Full real-time control over 115200 baud serial. Move individual joints, trigger preset sequences, or adjust speed on the fly
- **Preset Sequences** — Built-in pick-and-place demo and wave gesture that chain coordinated multi-joint movements
- **Adjustable Speed** — Step delay configurable from 1ms (fast) to 100ms (slow) per degree of travel
- **Modular Design** — Joint configuration is table-driven (`JointConfig` struct), making it trivial to adjust limits, pins, or home positions without touching motion logic

---

## Hardware Requirements

### Components

| Component | Qty | Purpose |
|-----------|-----|---------|
| Arduino Mega 2560 | 1 | Controller (Uno works but Mega recommended for pin count) |
| MG996R Servo | 3 | Base, Shoulder, Elbow (high-torque joints) |
| MG90S Servo | 3 | Wrist Pitch, Wrist Roll, Gripper (lighter joints) |
| 6V 5A DC Power Supply | 1 | Dedicated servo power — **do not use Arduino 5V** |
| Breadboard / PCB | 1 | Power distribution and signal routing |
| Jumper Wires | ~20 | Signal + power connections |

### Wiring Diagram

```
                    ┌──────────────────────┐
                    │   Arduino Mega 2560  │
                    │                      │
   Servo J0 ◄──────┤ Pin 2          GND ──┼──── Common GND ────┐
   Servo J1 ◄──────┤ Pin 3                │                    │
   Servo J2 ◄──────┤ Pin 4                │                    │
   Servo J3 ◄──────┤ Pin 5                │      ┌─────────┐   │
   Servo J4 ◄──────┤ Pin 6                │      │  6V 5A  │   │
   Servo J5 ◄──────┤ Pin 7                │      │  PSU    │   │
                    │                      │      │         │   │
                    └──────────────────────┘      │ V+ ─────┼───┼──► Servo V+
                                                  │ GND ────┼───┘
                                                  └─────────┘
```

> ⚠️ **Critical**: Servo motors draw significant current under load (up to 2.5A stall per MG996R). Always use a dedicated external power supply. Powering servos from the Arduino's 5V rail will cause brownouts, erratic behavior, and potential board damage.

---

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/Jai2379/6-DOF-Robotic-Arm.git
cd 6-DOF-Robotic-Arm
```

### 2. Open in Arduino IDE

- Open `robotic_arm.ino` in Arduino IDE (1.8+ or 2.0+)
- Select board: **Arduino Mega 2560** (or your board)
- Select the correct COM port

### 3. Upload

- Verify/compile: `Ctrl+R`
- Upload: `Ctrl+U`
- Open Serial Monitor: `Ctrl+Shift+M` → Set baud to **115200**

---

## Usage

### Serial Commands

All commands are sent via Serial Monitor at **115200 baud**. Commands are case-insensitive.

| Command | Action | Example |
|---------|--------|---------|
| `Jn:angle` | Move joint `n` to `angle` degrees | `J0:45` — rotate base to 45° |
| `HOME` | All joints return to home position | — |
| `PARK` | Fold arm into compact storage pose | — |
| `PICK` | Run pick-and-place demo sequence | — |
| `WAVE` | Friendly wave gesture | — |
| `GRIP:angle` | Set gripper angle (shortcut for J5) | `GRIP:73` — close gripper |
| `SPEED:ms` | Set interpolation step delay (1–100) | `SPEED:8` — fast movement |
| `POS` | Print all current joint positions | — |
| `HELP` | Display command reference | — |

### Example Session

```
═══════════════════════════════════════
  6-DOF Robotic Arm Controller v1.0
  Joints: Base | Shoulder | Elbow
          Wrist Pitch | Roll | Gripper
═══════════════════════════════════════
  ✓ J0 [Base] attached to pin 2 @ 90°
  ✓ J1 [Shoulder] attached to pin 3 @ 90°
  ✓ J2 [Elbow] attached to pin 4 @ 90°
  ✓ J3 [Wrist Pitch] attached to pin 5 @ 90°
  ✓ J4 [Wrist Roll] attached to pin 6 @ 90°
  ✓ J5 [Gripper] attached to pin 7 @ 10°

> J0:45
[MOVE] Base -> 45°

> GRIP:73
[MOVE] Gripper -> 73°

> POS
─── Current Joint Positions ───
  J0 [Base]: 45°  (0–180°)
  J1 [Shoulder]: 90°  (15–165°)
  J2 [Elbow]: 90°  (0–180°)
  J3 [Wrist Pitch]: 90°  (0–180°)
  J4 [Wrist Roll]: 90°  (0–180°)
  J5 [Gripper]: 73°  (10–73°)
───────────────────────────────

> PICK
[SEQ] Starting PICK sequence...
  -> Pre-pick position
  -> Reaching...
  -> Gripping
  -> Lifting
  -> Rotating to drop zone
  -> Releasing
[INFO] Moving to HOME position...
[INFO] HOME reached.
[SEQ] PICK sequence complete.
```

---

## Project Structure

```
6-DOF-Robotic-Arm/
├── robotic_arm.ino       # Main Arduino sketch — all control logic
├── README.md             # This file
└── solidworks/           # (Optional) SolidWorks assembly files
    ├── full_assembly.SLDASM
    ├── base.SLDPRT
    ├── lower_arm.SLDPRT
    ├── upper_arm.SLDPRT
    ├── wrist.SLDPRT
    └── gripper.SLDPRT
```

---

## How It Works

### Motion Interpolation

The arm doesn't move joints one at a time — it coordinates all 6 simultaneously. The `moveAllJoints()` function finds the joint with the largest angular travel and uses it as the reference. All other joints are linearly interpolated over the same number of steps:

```cpp
for (int step = 1; step <= maxDelta; step++) {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        int pos = startAngle[i] + (delta[i] * step) / maxDelta;
        servos[i].write(pos);
    }
    delay(stepDelayMs);
}
```

This produces smooth, coordinated motion where all joints arrive at their targets at the same time — regardless of how far each one needs to travel.

### Safety: Joint Limits

Every angle command is clamped to the mechanical limits defined in the `JointConfig` table before being sent to the servo. The shoulder joint (J1) is intentionally constrained to 15°–165° to prevent the parallel-linkage arm from colliding with the base platform — a constraint derived directly from the SolidWorks interference analysis.

---

## Roadmap

- [x] SolidWorks mechanical design — full 6-DOF assembly
- [x] Arduino servo control with smooth interpolation
- [x] Serial command interface
- [x] Preset motion sequences (PICK, WAVE)
- [ ] Inverse kinematics solver — move to XYZ coordinates instead of joint angles
- [ ] Joystick / gamepad input for real-time manual control
- [ ] Sensor integration — force-sensitive gripper feedback
- [ ] ROS integration for advanced path planning
- [ ] 3D-printed enclosures for servo mounts and wiring

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Mechanical Design | SolidWorks 2024 |
| Microcontroller | Arduino Mega 2560 |
| Language | C++ (Arduino framework) |
| Actuators | MG996R / MG90S Servo Motors |
| Communication | Serial (UART) @ 115200 baud |

---

## License

MIT License. See [LICENSE](LICENSE) for details.

---

<p align="center">
  <em>Designed and built by <strong>Jaiaditya Asudani</strong></em>
</p>
