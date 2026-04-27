/*
 * ═══════════════════════════════════════════════════════════════
 *  6-DOF Robotic Arm Controller
 *  Designed for: SolidWorks 6-DOF Articulated Arm w/ Parallel Linkage
 * ═══════════════════════════════════════════════════════════════
 *
 *  Joint Map (matching SolidWorks assembly):
 *
 *    Joint 0 — BASE ROTATION    (Servo on circular platform)
 *              Rotates entire arm in horizontal plane (yaw)
 *              Range: 0°–180°  |  Home: 90°
 *
 *    Joint 1 — SHOULDER         (Servo at base-to-lower-arm pivot)
 *              Tilts the lower parallel-linkage arm segment
 *              Range: 15°–165° |  Home: 90°
 *
 *    Joint 2 — ELBOW            (Servo at lower-arm-to-upper-arm pivot)
 *              Bends the upper arm segment relative to lower
 *              Range: 0°–180°  |  Home: 90°
 *
 *    Joint 3 — WRIST PITCH      (Servo at upper-arm-to-wrist pivot)
 *              Tilts the end-effector up/down
 *              Range: 0°–180°  |  Home: 90°
 *
 *    Joint 4 — WRIST ROLL       (Servo rotating the gripper mount)
 *              Rotates end-effector around its own axis
 *              Range: 0°–180°  |  Home: 90°
 *
 *    Joint 5 — GRIPPER          (Servo driving the claw mechanism)
 *              Opens and closes the claw/gripper
 *              Range: 10°–73°  |  Home: 10° (open)
 *
 *  Hardware:
 *    - 6x MG996R or equivalent servo motors (or MG90S for wrist/gripper)
 *    - Arduino Mega 2560 (recommended) or Uno
 *    - External 6V 5A power supply for servos (do NOT power from Arduino 5V)
 *    - Common GND between Arduino and servo PSU
 *
 *  Serial Commands (115200 baud):
 *    Jn:angle     — Move joint n to angle    (e.g. "J0:90")
 *    HOME         — Move all joints to home position
 *    PARK         — Move to folded park position
 *    PICK         — Execute pick-up demo sequence
 *    WAVE         — Friendly wave gesture
 *    GRIP:angle   — Set gripper (shortcut for J5)
 *    SPEED:ms     — Set step delay in ms (default 15)
 *    POS          — Print current joint positions
 *    HELP         — Print command list
 *
 * ═══════════════════════════════════════════════════════════════
 */

#include <Servo.h>

// ─── PIN CONFIGURATION ────────────────────────────────────────
// Use PWM-capable pins. On Mega: 2-13, 44-46. On Uno: 3,5,6,9,10,11.
#define PIN_BASE          2    // Joint 0 — Base rotation
#define PIN_SHOULDER      3    // Joint 1 — Shoulder
#define PIN_ELBOW         4    // Joint 2 — Elbow
#define PIN_WRIST_PITCH   5    // Joint 3 — Wrist pitch
#define PIN_WRIST_ROLL    6    // Joint 4 — Wrist roll
#define PIN_GRIPPER       7    // Joint 5 — Gripper

#define NUM_JOINTS        6

// ─── JOINT LIMITS (degrees) ──────────────────────────────────
// Based on mechanical constraints of the SolidWorks assembly.
// The parallel-linkage shoulder has tighter limits to avoid
// self-collision with the base platform.
struct JointConfig {
    const char* name;
    uint8_t pin;
    uint8_t minAngle;
    uint8_t maxAngle;
    uint8_t homeAngle;
};

const JointConfig JOINTS[NUM_JOINTS] = {
    { "Base",         PIN_BASE,         0,  180,  90 },   // J0
    { "Shoulder",     PIN_SHOULDER,    15,  165,  90 },   // J1
    { "Elbow",        PIN_ELBOW,        0,  180,  90 },   // J2
    { "Wrist Pitch",  PIN_WRIST_PITCH,  0,  180,  90 },   // J3
    { "Wrist Roll",   PIN_WRIST_ROLL,   0,  180,  90 },   // J4
    { "Gripper",      PIN_GRIPPER,     10,   73,  10 },   // J5
};

// ─── GLOBALS ──────────────────────────────────────────────────
Servo servos[NUM_JOINTS];
uint8_t currentAngle[NUM_JOINTS];   // Tracks current position of each joint
uint8_t targetAngle[NUM_JOINTS];    // Target position for interpolated moves
unsigned int stepDelayMs = 15;      // ms between each 1° step (controls speed)

// ─── UTILITY: Clamp angle to joint limits ─────────────────────
uint8_t clampAngle(uint8_t joint, int angle) {
    if (angle < JOINTS[joint].minAngle) return JOINTS[joint].minAngle;
    if (angle > JOINTS[joint].maxAngle) return JOINTS[joint].maxAngle;
    return (uint8_t)angle;
}

// ─── MOVE: Single joint smooth interpolation ──────────────────
// Moves one joint from current position to target, 1° at a time.
void moveJoint(uint8_t joint, uint8_t target) {
    if (joint >= NUM_JOINTS) return;
    target = clampAngle(joint, target);

    int current = currentAngle[joint];
    int step = (target > current) ? 1 : -1;

    while (current != target) {
        current += step;
        servos[joint].write(current);
        delay(stepDelayMs);
    }
    currentAngle[joint] = target;
}

// ─── MOVE: All joints simultaneously (coordinated motion) ─────
// Interpolates all joints toward their targets at roughly the
// same rate so the arm traces a smoother path in joint-space.
void moveAllJoints(uint8_t targets[NUM_JOINTS]) {
    // Clamp all targets
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        targets[i] = clampAngle(i, targets[i]);
    }

    // Find the joint with the largest travel distance
    int maxDelta = 0;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        int delta = abs((int)targets[i] - (int)currentAngle[i]);
        if (delta > maxDelta) maxDelta = delta;
    }

    if (maxDelta == 0) return; // Already at target

    // Step all joints proportionally
    for (int step = 1; step <= maxDelta; step++) {
        for (uint8_t i = 0; i < NUM_JOINTS; i++) {
            int delta = (int)targets[i] - (int)currentAngle[i];
            // Linear interpolation: position = start + (delta * step / maxDelta)
            int pos = (int)currentAngle[i] + (delta * step) / maxDelta;
            // Avoid redundant writes — only if the rounded position changed
            servos[i].write(pos);
        }
        delay(stepDelayMs);
    }

    // Finalize positions
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        currentAngle[i] = targets[i];
        servos[i].write(targets[i]);
    }
}

// ─── PRESET: Home position ────────────────────────────────────
// Arm stands upright, centered, gripper open.
void goHome() {
    Serial.println(F("[INFO] Moving to HOME position..."));
    uint8_t home[NUM_JOINTS];
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        home[i] = JOINTS[i].homeAngle;
    }
    moveAllJoints(home);
    Serial.println(F("[INFO] HOME reached."));
}

// ─── PRESET: Park (folded compact position) ──────────────────
// Arm folds down for storage/transport. Minimizes profile.
void goPark() {
    Serial.println(F("[INFO] Moving to PARK position..."));
    uint8_t park[NUM_JOINTS] = {
        90,   // Base centered
        150,  // Shoulder folded down toward base
        30,   // Elbow tucked in
        90,   // Wrist neutral
        90,   // Roll neutral
        10    // Gripper open
    };
    moveAllJoints(park);
    Serial.println(F("[INFO] PARK reached."));
}

// ─── SEQUENCE: Pick-up demo ──────────────────────────────────
// Demonstrates reaching forward, gripping, lifting, and returning.
void pickSequence() {
    Serial.println(F("[SEQ] Starting PICK sequence..."));

    // 1. Move to pre-pick position (arm extended forward, low)
    uint8_t prePick[NUM_JOINTS] = { 90, 60, 120, 130, 90, 10 };
    Serial.println(F("  -> Pre-pick position"));
    moveAllJoints(prePick);
    delay(300);

    // 2. Lower to object (shoulder drops, wrist adjusts)
    uint8_t reach[NUM_JOINTS] = { 90, 45, 135, 140, 90, 10 };
    Serial.println(F("  -> Reaching..."));
    moveAllJoints(reach);
    delay(300);

    // 3. Close gripper
    Serial.println(F("  -> Gripping"));
    moveJoint(5, 73);  // Gripper closed
    delay(500);

    // 4. Lift object (shoulder raises, elbow straightens)
    uint8_t lift[NUM_JOINTS] = { 90, 90, 90, 90, 90, 73 };
    Serial.println(F("  -> Lifting"));
    moveAllJoints(lift);
    delay(300);

    // 5. Rotate to drop position
    uint8_t drop[NUM_JOINTS] = { 45, 90, 90, 90, 90, 73 };
    Serial.println(F("  -> Rotating to drop zone"));
    moveAllJoints(drop);
    delay(300);

    // 6. Release
    Serial.println(F("  -> Releasing"));
    moveJoint(5, 10);  // Gripper open
    delay(300);

    // 7. Return home
    goHome();
    Serial.println(F("[SEQ] PICK sequence complete."));
}

// ─── SEQUENCE: Wave gesture ──────────────────────────────────
void waveSequence() {
    Serial.println(F("[SEQ] Waving..."));

    // Raise arm
    uint8_t raised[NUM_JOINTS] = { 90, 90, 45, 90, 90, 73 };
    moveAllJoints(raised);
    delay(200);

    // Wave the wrist back and forth 3 times
    for (int i = 0; i < 3; i++) {
        moveJoint(3, 45);
        moveJoint(3, 135);
    }
    moveJoint(3, 90);
    delay(200);

    goHome();
    Serial.println(F("[SEQ] Wave complete."));
}

// ─── PRINT: Current joint positions ──────────────────────────
void printPositions() {
    Serial.println(F("─── Current Joint Positions ───"));
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        Serial.print(F("  J"));
        Serial.print(i);
        Serial.print(F(" ["));
        Serial.print(JOINTS[i].name);
        Serial.print(F("]: "));
        Serial.print(currentAngle[i]);
        Serial.print(F("°  ("));
        Serial.print(JOINTS[i].minAngle);
        Serial.print(F("–"));
        Serial.print(JOINTS[i].maxAngle);
        Serial.println(F("°)"));
    }
    Serial.println(F("───────────────────────────────"));
}

// ─── PRINT: Help / command reference ─────────────────────────
void printHelp() {
    Serial.println(F(""));
    Serial.println(F("╔═══════════════════════════════════════╗"));
    Serial.println(F("║   6-DOF Robotic Arm — Command List    ║"));
    Serial.println(F("╠═══════════════════════════════════════╣"));
    Serial.println(F("║ Jn:angle  Move joint n to angle       ║"));
    Serial.println(F("║           e.g. J0:90, J5:73           ║"));
    Serial.println(F("║ HOME      All joints to home          ║"));
    Serial.println(F("║ PARK      Fold arm for storage        ║"));
    Serial.println(F("║ PICK      Pick-up demo sequence       ║"));
    Serial.println(F("║ WAVE      Wave gesture                ║"));
    Serial.println(F("║ GRIP:n    Set gripper angle            ║"));
    Serial.println(F("║ SPEED:n   Set step delay (ms)         ║"));
    Serial.println(F("║ POS       Print current positions     ║"));
    Serial.println(F("║ HELP      Show this message           ║"));
    Serial.println(F("╚═══════════════════════════════════════╝"));
    Serial.println(F(""));
}

// ─── SERIAL COMMAND PARSER ────────────────────────────────────
void parseCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "HOME") {
        goHome();
    }
    else if (cmd == "PARK") {
        goPark();
    }
    else if (cmd == "PICK") {
        pickSequence();
    }
    else if (cmd == "WAVE") {
        waveSequence();
    }
    else if (cmd == "POS") {
        printPositions();
    }
    else if (cmd == "HELP") {
        printHelp();
    }
    else if (cmd.startsWith("SPEED:")) {
        int val = cmd.substring(6).toInt();
        if (val >= 1 && val <= 100) {
            stepDelayMs = val;
            Serial.print(F("[INFO] Step delay set to "));
            Serial.print(stepDelayMs);
            Serial.println(F(" ms"));
        } else {
            Serial.println(F("[ERR] Speed must be 1–100 ms"));
        }
    }
    else if (cmd.startsWith("GRIP:")) {
        int angle = cmd.substring(5).toInt();
        Serial.print(F("[MOVE] Gripper -> "));
        Serial.println(angle);
        moveJoint(5, angle);
    }
    else if (cmd.startsWith("J") && cmd.indexOf(':') > 0) {
        // Parse "Jn:angle"
        int colonIdx = cmd.indexOf(':');
        int joint = cmd.substring(1, colonIdx).toInt();
        int angle = cmd.substring(colonIdx + 1).toInt();

        if (joint >= 0 && joint < NUM_JOINTS) {
            Serial.print(F("[MOVE] "));
            Serial.print(JOINTS[joint].name);
            Serial.print(F(" -> "));
            Serial.print(clampAngle(joint, angle));
            Serial.println(F("°"));
            moveJoint(joint, angle);
        } else {
            Serial.println(F("[ERR] Invalid joint index (0–5)"));
        }
    }
    else {
        Serial.print(F("[ERR] Unknown command: "));
        Serial.println(cmd);
        Serial.println(F("       Type HELP for command list."));
    }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }  // Wait for serial on boards with native USB

    Serial.println(F(""));
    Serial.println(F("═══════════════════════════════════════"));
    Serial.println(F("  6-DOF Robotic Arm Controller v1.0"));
    Serial.println(F("  Joints: Base | Shoulder | Elbow"));
    Serial.println(F("          Wrist Pitch | Roll | Gripper"));
    Serial.println(F("═══════════════════════════════════════"));

    // Attach all servos and move to home position
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        servos[i].attach(JOINTS[i].pin);
        currentAngle[i] = JOINTS[i].homeAngle;
        servos[i].write(currentAngle[i]);
        Serial.print(F("  ✓ J"));
        Serial.print(i);
        Serial.print(F(" ["));
        Serial.print(JOINTS[i].name);
        Serial.print(F("] attached to pin "));
        Serial.print(JOINTS[i].pin);
        Serial.print(F(" @ "));
        Serial.print(currentAngle[i]);
        Serial.println(F("°"));
    }

    Serial.println(F(""));
    Serial.println(F("  All servos initialized at HOME."));
    Serial.println(F("  Type HELP for command list."));
    Serial.println(F("═══════════════════════════════════════"));
    Serial.println(F(""));
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        parseCommand(cmd);
    }
}
