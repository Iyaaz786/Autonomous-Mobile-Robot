# Autonomous Mobile Robot: Multi-Modal Navigation & Path Planning

## 🎯 Overview
[cite_start]This repository contains the firmware for a 4-wheeled autonomous robot capable of complex navigation in dynamic environments[cite: 128, 130]. [cite_start]The system implements a modular state-machine architecture to switch between various robotic behaviors, ranging from simple reactive obstacle avoidance to complex path-planning algorithms like the **Bug Algorithm**[cite: 248].

## 🧠 Navigation Logic (The "Bug" Algorithm)
[cite_start]The highlight of this project is **Mode 4: The Bug Algorithm**[cite: 248]. This algorithm allows the robot to navigate towards a goal while handling unexpected obstacles by transitioning through two primary states:

1. [cite_start]**State 0 (Goal Seeking):** The robot moves forward towards a predefined target[cite: 294, 296].
2. [cite_start]**State 1 (Wall Following):** Upon detecting an obstacle within **20cm**, the robot triggers an audible alert and switches to a wall-following behavior to "circumnavigate" the object[cite: 281, 295].
3. [cite_start]**Recovery:** Once the front path is clear for a set duration, the robot resets its state to continue towards the goal[cite: 298, 299].

---

## 🛠️ Hardware Specifications
| Component | Specification / Pin | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino | [cite_start]Central Logic Processing[cite: 130]. |
| **Distance Sensor** | HC-SR04 Ultrasonic | [cite_start]Real-time obstacle detection (Trig: 10, Echo: 2)[cite: 249]. |
| **Actuator** | Micro Servo (SG90) | [cite_start]Provides 180° environmental scanning capability[cite: 249]. |
| **Line Sensor** | 5-Channel IR Array | [cite_start]High-precision surface tracking (A0-A4)[cite: 249]. |
| **Drive System** | L298N / DC Motors | [cite_start]Differential drive with PWM speed control[cite: 248]. |

---

## 🔬 Perception & Sensor Calibration
[cite_start]During development, I identified a critical limitation with standard Infrared (IR) sensors, which often failed to detect objects at certain angles[cite: 132, 133]. [cite_start]I resolved this by transitioning to an **Ultrasonic-first perception model**[cite: 135].

### **Sensor Accuracy Analysis**
[cite_start]I performed rigorous testing to map "Actual Distance" vs. "Sensor Reading" to ensure navigation reliability[cite: 136, 145].

*(Insert your Graph Image here from Portfolio Page 8)*

---

## 💻 Code Architecture
The software is organized into functional blocks to ensure scalability:
* [cite_start]**`obstacleAvoidance()`**: Implements "Scan-Compare-Turn" logic[cite: 263]. [cite_start]If the front path is blocked, the robot scans 30° right and 150° left to find the optimal path[cite: 261].
* [cite_start]**`wallFollowing()`**: Uses distance thresholds (**25cm**) and tolerances to maintain a steady course parallel to a wall[cite: 249, 280].
* [cite_start]**`lineFollowing()`**: Utilizes a 5-sensor array to handle curves, sharp turns, and intersections[cite: 270, 271].

### **Featured Snippet: State Switching**
```cpp
void bugAlgorithm() {
  if (bugState == 0) {
    if (frontDistance < OBSTACLE_STOP_DISTANCE) {
      bugState = 1; // Transition to Wall Follow
      buzz();
      stopMotors();
    } else {
      moveForward(NORMAL_SPEED);
    }
  } else {
    wallFollowing(); // Perimeter navigation
  }
}
