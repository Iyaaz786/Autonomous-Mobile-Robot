# Autonomous Mobile Robot (AMR)

> University project — design, build, and program an Arduino-based autonomous robot car featuring a multi-modal, sensor-driven navigation system.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Navigation Modes](#navigation-modes)
3. [Modular Architecture — State Machine](#modular-architecture--state-machine)
4. [Bug Algorithm](#bug-algorithm)
5. [Sensor Integration](#sensor-integration)
6. [Hardware Safety](#hardware-safety)
7. [Pulse-Width Modulation (PWM) for Speed Control](#pulse-width-modulation-pwm-for-speed-control)
8. [Reactive Navigation](#reactive-navigation)
9. [Hysteresis / Tolerance in Wall Following](#hysteresis--tolerance-in-wall-following)
10. [Folder Structure](#folder-structure)
11. [Hardware Bill of Materials](#hardware-bill-of-materials)
12. [Wiring Overview](#wiring-overview)
13. [Getting Started](#getting-started)
14. [Images](#images)
15. [License](#license)

---

## Project Overview

This project implements a **multi-modal control system** for an Arduino-based Autonomous Mobile Robot (AMR). The robot can navigate its environment using four distinct behaviours — Obstacle Avoidance, Line Following, Wall Following, and the Bug Algorithm — all managed by a single, easily configurable state machine. The active mode is selected by changing one constant in the source code, making the architecture highly modular and extensible.

---

## Navigation Modes

| Mode | Constant | Description |
|------|----------|-------------|
| **Obstacle Avoidance** | `MODE_OBSTACLE_AVOID` | Scans ahead with the ultrasonic sensor; stops, buzzes, then steers around any object within the safety threshold. |
| **Line Following** | `MODE_LINE_FOLLOW` | Uses left and right IR sensors to track a high-contrast line on the floor in real time. |
| **Wall Following** | `MODE_WALL_FOLLOW` | Maintains a constant lateral distance to a wall using hysteresis-based correction (right-hand rule). |
| **Bug Algorithm** | `MODE_BUG_ALGORITHM` | Combines goal-directed travel with reactive obstacle circumnavigation — see [Bug Algorithm](#bug-algorithm). |

---

## Modular Architecture — State Machine

The entire navigation logic is gated by a top-level `switch (MODE)` statement inside `loop()`:

```cpp
switch (MODE) {
    case MODE_OBSTACLE_AVOID:  obstacleAvoidance(); break;
    case MODE_LINE_FOLLOW:     lineFollowing();     break;
    case MODE_WALL_FOLLOW:     wallFollowing();     break;
    case MODE_BUG_ALGORITHM:   bugAlgorithm();      break;
    default:                   motorsStop();        break;
}
```

Each behaviour is self-contained in its own function. Adding a new navigation mode requires only:

1. Defining a new `MODE_*` constant.
2. Implementing a corresponding function.
3. Adding one `case` to the `switch` block.

This **state-machine approach** keeps the codebase clean, testable, and easy to extend without touching unrelated logic.

---

## Bug Algorithm

The Bug Algorithm is a classical reactive navigation strategy that guarantees goal reachability in environments with convex and concave obstacles.

### States

```
┌────────────────────┐   obstacle detected   ┌─────────────────────┐
│  State 0           │ ───────────────────►  │  State 1            │
│  Goal Seeking      │                        │  Wall Following     │
│  (drive to goal)   │ ◄───────────────────  │  (circumnavigate)   │
└────────────────────┘   path clear ahead     └─────────────────────┘
```

| State | `bugState` value | Behaviour |
|-------|-----------------|-----------|
| **Goal Seeking** | `BUG_STATE_GOAL_SEEK` (0) | Robot drives straight toward the target goal heading. Motors run at `BASE_SPEED` using PWM. |
| **Wall Following** | `BUG_STATE_WALL_FOLLOW` (1) | Robot switches to the wall-following behaviour to circumnavigate the detected obstacle. |

**Transition logic:**

- **State 0 → State 1:** When the ultrasonic sensor measures `frontDist < OBSTACLE_DIST_CM`, the motors stop, the buzzer fires, and `bugState` is set to `BUG_STATE_WALL_FOLLOW`.
- **State 1 → State 0:** After each wall-following iteration the sensor checks the forward distance again. When `frontDist >= OBSTACLE_DIST_CM * 2` (i.e. the path ahead is clearly unobstructed), `bugState` is reset to `BUG_STATE_GOAL_SEEK`.

---

## Sensor Integration

### HC-SR04 Ultrasonic Sensor on a Micro-Servo Head

The HC-SR04 is mounted on a **micro-servo** that rotates the sensor through 180 °, allowing the robot to build a simple angular distance map without physically turning.

```
         ┌─────────────┐
         │  HC-SR04    │   ← mounted on micro-servo horn
         │  TRIG  ECHO │
         └──────┬──────┘
                │ servo shaft
         ┌──────┴──────┐
         │  Micro-Servo │  (signal → SERVO_PIN 11)
         └─────────────┘
```

**Scanning procedure (`scanAt(angle)`):**

1. `setServoAngle(angle)` — generates the correct PWM pulse width for the desired angle  
   (`pulseWidth = map(angle, 0, 180, 544, 2400)` µs).
2. A 150 ms settling delay ensures the servo has physically reached the target position.
3. `readDistanceCm()` fires a 10 µs TRIG pulse and times the ECHO return, converting the round-trip duration to centimetres: `distance = duration / 58`.

Typical scan pattern used during Obstacle Avoidance:

| Servo Angle | Direction |
|-------------|-----------|
| 180 ° | Left |
| 90 ° | Ahead |
| 0 ° | Right |

---

## Hardware Safety

Two independent safety mechanisms protect the robot and its surroundings:

### 1. Audible Buzzer Alert — `buzz(durationMs)`

```cpp
void buzz(int durationMs) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(durationMs);
    digitalWrite(BUZZER_PIN, LOW);
}
```

The buzzer fires in every code path where an obstacle is detected or a mode transition occurs, providing immediate audible feedback.

### 2. Automatic Motor Stop — `motorsStop()`

```cpp
void motorsStop() {
    analogWrite(ENA, 0); analogWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
```

Before any corrective manoeuvre (turn, back-up, mode switch), both motors are halted by zeroing the PWM duty cycle on the enable pins and pulling all direction pins LOW, eliminating coasting drift.

---

## Pulse-Width Modulation (PWM) for Speed Control

Motor speed is controlled using Arduino's `analogWrite()`, which generates a **Pulse-Width Modulated** signal on the enable pins (`ENA`, `ENB`) of the L298N motor driver:

```
Duty cycle 0 %   ─────────────────  (motor stopped)
Duty cycle 59 %  ████████─────────  (BASE_SPEED = 150 / 255 ≈ 59 %)
Duty cycle 100 % █████████████████  (full speed)
```

Adjusting `BASE_SPEED` (0–255) scales motor torque proportionally, enabling smooth acceleration profiles and differential steering without additional circuitry.

---

## Reactive Navigation

All four navigation modes implement **Reactive Navigation**: the robot continuously reads sensors and immediately translates those readings into motor commands without maintaining a global map.

Key properties of the reactive approach used here:

- **Sensor–Action loops run every iteration of `loop()`** — latency is bounded by sensor read time (≈ 5–30 ms for HC-SR04).
- **No pre-planned path** — behaviour emerges from local sensor data alone.
- **Graceful degradation** — if a sensor returns an out-of-range value (0 from HC-SR04 timeout), the robot defaults to a safe action (`motorsStop()`).

---

## Hysteresis / Tolerance in Wall Following

Naïve wall following using a single threshold causes rapid, oscillating corrections (motor chatter). This implementation uses a **hysteresis band** of ±`WALL_TOLERANCE_CM` around the target distance:

```
Wall distance (cm)
  ^
  │    WALL_TARGET + WALL_TOLERANCE  ── steer right threshold
  │    ──────────────────────────────
  │    WALL_TARGET                   ── ideal distance
  │    ──────────────────────────────
  │    WALL_TARGET - WALL_TOLERANCE  ── steer left threshold
  │
  └──────────────────────────────────► time
```

| Condition | Action |
|-----------|--------|
| `wallDist > WALL_TARGET + WALL_TOLERANCE` | `turnRight` — too far from wall |
| `wallDist < WALL_TARGET - WALL_TOLERANCE` | `turnLeft` — too close to wall |
| Otherwise (inside hysteresis band) | `motorsForward` — no correction needed |

This prevents unnecessary corrections when the robot is already within an acceptable corridor, significantly reducing motor wear and improving straight-line stability.

---

## Folder Structure

```
Autonomous-Mobile-Robot/
├── src/
│   └── Bug.ino                  ← Main Arduino sketch (all navigation modes)
├── images/
│   ├── robot_assembly.md      ← Placeholder: robot assembly photographs
│   └── sensor_calibration.md ← Placeholder: sensor calibration graphs
└── README.md
```

---

## Hardware Bill of Materials

| Component | Quantity | Notes |
|-----------|----------|-------|
| Arduino Uno / Mega | 1 | Main microcontroller |
| L298N Dual H-Bridge Motor Driver | 1 | PWM speed + direction control |
| DC Gear Motors + Wheels | 2 | Left & right drive |
| HC-SR04 Ultrasonic Sensor | 1 | Distance measurement (2–400 cm) |
| Micro-Servo (e.g. SG90) | 1 | Sensor head rotation |
| IR Obstacle Sensors | 2 | Left / right line following |
| Passive Buzzer | 1 | Audible safety alerts |
| Robot Chassis | 1 | Acrylic or 3-D printed |
| Li-Po / AA Battery Pack | 1 | 7–12 V supply |
| Jumper Wires & Breadboard | — | Prototyping |

---

## Wiring Overview

| Signal | Arduino Pin |
|--------|-------------|
| HC-SR04 TRIG | D9 |
| HC-SR04 ECHO | D10 |
| Servo Signal | D11 |
| Motor L — ENA (PWM) | D5 |
| Motor L — IN1 | D4 |
| Motor L — IN2 | D3 |
| Motor R — ENB (PWM) | D6 |
| Motor R — IN3 | D7 |
| Motor R — IN4 | D8 |
| IR Sensor Left | A0 |
| IR Sensor Right | A1 |
| Buzzer | D12 |

---

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) 1.8+ or Arduino CLI
- Arduino Uno or Mega board

### Upload

1. Clone this repository:
   ```bash
   git clone https://github.com/Iyaaz786/Autonomous-Mobile-Robot.git
   cd Autonomous-Mobile-Robot
   ```
2. Open `src/Bug.ino` in the Arduino IDE.
3. Select your board and COM port under **Tools**.
4. Change the `MODE` constant at the top of the file to select the desired navigation behaviour.
5. Click **Upload** (or run `arduino-cli compile --upload`).
6. Open the Serial Monitor at **9600 baud** to see initialisation messages.

---

## Images

> Photographs and graphs will be added here once the robot has been assembled and calibrated.

| Asset | Description |
|-------|-------------|
| `images/robot_assembly_front.jpg` | Front view — servo head and sensor |
| `images/robot_assembly_top.jpg` | Top view — Arduino + wiring layout |
| `images/ultrasonic_calibration.png` | HC-SR04 measured vs. actual distance |
| `images/servo_angle_map.png` | Servo pulse-width vs. rotation angle |

---

## License

This project is released for educational purposes. See [LICENSE](LICENSE) for details.
