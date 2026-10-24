# 6-DOF Robotic Arm: Arduino Servo Controller

SolidWorks design of an articulated arm with a parallel-linkage lower arm, plus Arduino firmware that drives its six servos (five positioning axes and a gripper) through clamped, coordinated joint-space moves over a serial command protocol.

## Kinematic chain

```
J0 base yaw ─► J1 shoulder ─► J2 elbow ─► J3 wrist pitch ─► J4 wrist roll ─► J5 gripper
   (platform)    (parallel-linkage lower arm)                                   (claw)
```

| Joint | Function | Pin | Range | Home | Servo |
|:--|:--|:--|:--|:--|:--|
| J0 | Base yaw | 2 | 0–180° | 90° | MG996R |
| J1 | Shoulder | 3 | 15–165° | 90° | MG996R |
| J2 | Elbow | 4 | 0–180° | 90° | MG996R |
| J3 | Wrist pitch | 5 | 0–180° | 90° | MG90S |
| J4 | Wrist roll | 6 | 0–180° | 90° | MG90S |
| J5 | Gripper | 7 | 10–73° | 10° (open) | MG90S |

J1 is limited to 15–165° so the parallel linkage cannot collide with the base platform in the assembly. The gripper range maps 10° to fully open and 73° to fully closed.

## Firmware design

**Table-driven joint config.** All per-joint data lives in one `const JointConfig JOINTS[6]` array (`name`, `pin`, `minAngle`, `maxAngle`, `homeAngle`). Limits, pins and home poses change in one place without touching motion code.

**Clamping.** Every target passes through `clampAngle(joint, angle)` before it reaches a servo, including serial input and preset poses.

**Coordinated interpolation.** `moveAllJoints(targets[])` finds the largest angular travel `Δmax` across the six joints and steps every joint along a straight line in joint space:

```cpp
for (int step = 1; step <= maxDelta; step++) {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        int pos = currentAngle[i] + (delta[i] * step) / maxDelta;
        servos[i].write(pos);
    }
    delay(stepDelayMs);
}
```

All joints start and finish together. Move time is `Δmax × stepDelayMs`, so a 90° move at the default 15 ms/step takes 1.35 s plus servo lag. Integer division keeps the loop free of floating point on the AVR. A final write lands each joint exactly on target.

**Single-joint moves.** `moveJoint(j, target)` steps one joint 1° at a time. The gripper uses it so grip and release happen independently of arm motion.

**Speed control.** `stepDelayMs` defaults to 15 ms and accepts 1–100 ms at runtime.

## Serial protocol

115200 baud, newline-terminated, case-insensitive. Input is trimmed and upper-cased before parsing.

| Command | Effect |
|:--|:--|
| `Jn:angle` | Move joint `n` (0–5) to `angle`, clamped to its limits |
| `GRIP:angle` | Gripper shortcut for `J5` |
| `SPEED:ms` | Set step delay, 1–100 ms |
| `HOME` | All joints to home pose |
| `PARK` | Folded pose `{90, 150, 30, 90, 90, 10}` |
| `PICK` | Pick-and-place sequence |
| `WAVE` | Wrist-pitch oscillation 45°↔135°, three cycles |
| `POS` | Print each joint's current angle and limits |
| `HELP` | Print command table |

Responses carry a tag: `[MOVE]`, `[SEQ]`, `[INFO]` or `[ERR]`.

`PICK` runs these poses as `{J0, J1, J2, J3, J4, J5}`:

| Phase | Pose |
|:--|:--|
| Pre-pick | `{90, 60, 120, 130, 90, 10}` |
| Reach | `{90, 45, 135, 140, 90, 10}` |
| Grip | J5 → 73 |
| Lift | `{90, 90, 90, 90, 90, 73}` |
| Rotate to drop | `{45, 90, 90, 90, 90, 73}` |
| Release | J5 → 10, then `HOME` |

## Hardware

| Component | Qty | Notes |
|:--|:--|:--|
| Arduino Mega 2560 | 1 | Uno also works. PWM on pins 2–7 |
| MG996R | 3 | J0–J2, high-torque joints |
| MG90S | 3 | J3–J5 |
| 6 V 5 A supply | 1 | Servo rail. Shares ground with the Arduino |

An MG996R draws up to about 2.5 A at stall. The servos run from the external 6 V rail, never the Arduino's 5 V regulator, which browns out under that load.

```
Arduino Mega            6 V / 5 A PSU
  D2..D7 ── signal ──►  servos J0..J5
  GND ─────────────────  GND ──► servo GND
                         V+  ──► servo V+
```

## CAD

SolidWorks part and assembly files, in the repo root:

| File | Part |
|:--|:--|
| `Base (part 1)].SLDPRT` | Fixed base |
| `Rotational Base (Part 2).SLDPRT` | Yaw platform (J0) |
| `Arm Link (Part3).SLDPRT` | Lower arm link |
| `Arm Link 2 (Part 4).SLDPRT` | Upper arm link |
| `Gripper Base (Part 5).SLDPRT` | Gripper mount |
| `Gripper Link (Part 6).SLDPRT` | Gripper linkage |
| `Gripper Finger (Part 7).SLDPRT` | Finger |
| `Gripper Joint (Part 8).SLDPRT` | Finger pivot |
| `Full scale assembly.SLDASM` | Full assembly |

## Build

1. Open `robotic_arm.ino` in Arduino IDE 1.8+ or 2.x. The only dependency is the bundled `Servo` library.
2. Select **Arduino Mega 2560** and the board's port, then upload.
3. Open Serial Monitor at **115200** baud with newline line endings and send `HELP`.

## Next

- Inverse kinematics so commands can target XYZ positions instead of joint angles.
- Non-blocking motion with `millis()` scheduling. Moves currently block on `delay()`, so serial input waits until a move finishes.
- Trapezoidal velocity profiles to cut the jerk at the start and end of each move.
