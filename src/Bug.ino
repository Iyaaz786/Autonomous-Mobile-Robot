/*
 * Bug.ino — Arduino Autonomous Mobile Robot
 * ==========================================
 * Implements a multi-modal control system with the following navigation
 * behaviours selectable via the MODE switch-case state machine:
 *
 *   MODE 0  – Obstacle Avoidance
 *   MODE 1  – Line Following
 *   MODE 2  – Wall Following
 *   MODE 3  – Bug Algorithm (Goal Seeking + Wall Following)
 *
 * Hardware
 * --------
 *   • Arduino Uno / Mega
 *   • HC-SR04 Ultrasonic sensor mounted on a micro-servo 'head'
 *   • L298N (or compatible) dual H-bridge motor driver
 *   • IR sensors (left / right) for line following
 *   • Passive buzzer for audible safety alerts
 *
 * Key Concepts
 * ------------
 *   • Pulse-Width Modulation (PWM) for speed control via analogWrite()
 *   • Reactive Navigation — sensor-driven, real-time path adjustment
 *   • Hysteresis / Tolerance in wall following to prevent oscillation
 *   • Bug Algorithm: switches between State 0 (Goal Seeking) and
 *     State 1 (Wall Following) upon obstacle detection
 */

#include <Servo.h>

// ─── Pin Definitions ───────────────────────────────────────────────────────

// Ultrasonic sensor
#define TRIG_PIN   9
#define ECHO_PIN   10

// Servo (sensor head)
#define SERVO_PIN  11

// Motor driver — Left motor
#define ENA        5   // PWM speed control
#define IN1        4
#define IN2        3

// Motor driver — Right motor
#define ENB        6   // PWM speed control
#define IN3        7
#define IN4        8

// IR line sensors
#define IR_LEFT    A0
#define IR_RIGHT   A1

// Buzzer
#define BUZZER_PIN 12

// ─── Thresholds ────────────────────────────────────────────────────────────

#define OBSTACLE_DIST_CM        20       // Minimum safe distance (cm)
#define WALL_TARGET_CM          15       // Desired wall-following distance (cm)
#define WALL_TOLERANCE_CM        3       // Hysteresis band (±cm) — prevents oscillation
#define BASE_SPEED             150       // Default PWM duty cycle (0–255)
#define ULTRASONIC_TIMEOUT_US  30000UL  // HC-SR04 echo timeout (µs) — ~5 m max range
#define BUG_CLEARANCE_MULTIPLIER 2      // Path multiplier before resuming goal seeking
#define SERVO_SETTLE_MS         80       // Time (ms) for servo to reach target angle

// ─── Navigation Modes ──────────────────────────────────────────────────────

#define MODE_OBSTACLE_AVOID  0
#define MODE_LINE_FOLLOW     1
#define MODE_WALL_FOLLOW     2
#define MODE_BUG_ALGORITHM   3

// Change this constant to select the active navigation behaviour.
const int MODE = MODE_BUG_ALGORITHM;

// ─── Bug Algorithm State ───────────────────────────────────────────────────

#define BUG_STATE_GOAL_SEEK   0   // Drive straight toward goal
#define BUG_STATE_WALL_FOLLOW 1   // Circumnavigate obstacle

int bugState = BUG_STATE_GOAL_SEEK;

// ─── Servo Instance ────────────────────────────────────────────────────────

Servo headServo;

// ─── Servo Helper ──────────────────────────────────────────────────────────

void setServoAngle(int angle) {
    headServo.write(angle);
}

// ─── Ultrasonic Helper ─────────────────────────────────────────────────────

long readDistanceCm() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
    return duration / 58L; // cm
}

// Scan at a given servo angle and return distance (cm)
long scanAt(int angle) {
    setServoAngle(angle);
    delay(SERVO_SETTLE_MS); // allow servo to reach position
    return readDistanceCm();
}

// ─── Motor Control ─────────────────────────────────────────────────────────

void motorsForward(int speed) {
    analogWrite(ENA, speed); analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void motorsStop() {
    analogWrite(ENA, 0); analogWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void turnLeft(int speed) {
    analogWrite(ENA, speed); analogWrite(ENB, speed);
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
    analogWrite(ENA, speed); analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

// ─── Safety Buzzer ─────────────────────────────────────────────────────────

void buzz(int durationMs) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(durationMs);
    digitalWrite(BUZZER_PIN, LOW);
}

// ─── Navigation Behaviours ─────────────────────────────────────────────────

// MODE 0: Obstacle Avoidance
void obstacleAvoidance() {
    setServoAngle(90); // look straight ahead
    delay(100);
    long dist = readDistanceCm();

    if (dist > 0 && dist < OBSTACLE_DIST_CM) {
        motorsStop();
        buzz(200);

        // Scan left and right to choose the clearer path
        long distLeft  = scanAt(180);
        long distRight = scanAt(0);
        setServoAngle(90);

        if (distLeft > distRight) {
            turnLeft(BASE_SPEED);
        } else {
            turnRight(BASE_SPEED);
        }
        delay(400);
    } else {
        motorsForward(BASE_SPEED);
    }
}

// MODE 1: Line Following (IR sensors)
void lineFollowing() {
    int left  = digitalRead(IR_LEFT);
    int right = digitalRead(IR_RIGHT);

    if (left == LOW && right == LOW) {
        motorsForward(BASE_SPEED);        // on line
    } else if (left == HIGH && right == LOW) {
        turnLeft(BASE_SPEED);             // drift right → steer left
    } else if (left == LOW && right == HIGH) {
        turnRight(BASE_SPEED);            // drift left  → steer right
    } else {
        motorsStop();                     // lost line
    }
}

// MODE 2: Wall Following (right-hand rule with hysteresis)
void wallFollowing() {
    setServoAngle(0); // scan to the right
    delay(100);
    long wallDist = readDistanceCm();

    setServoAngle(90); // look ahead
    delay(100);
    long frontDist = readDistanceCm();

    if (frontDist > 0 && frontDist < OBSTACLE_DIST_CM) {
        // Obstacle ahead — turn left
        motorsStop();
        buzz(100);
        turnLeft(BASE_SPEED);
        delay(400);
        return;
    }

    // Hysteresis band: only correct if outside tolerance window
    long diff = wallDist - WALL_TARGET_CM;
    if (diff > WALL_TOLERANCE_CM) {
        turnRight(BASE_SPEED);  // too far from wall
        delay(100);
    } else if (diff < -WALL_TOLERANCE_CM) {
        turnLeft(BASE_SPEED);   // too close to wall
        delay(100);
    } else {
        motorsForward(BASE_SPEED); // inside tolerance — go straight
    }
}

// MODE 3: Bug Algorithm
//   State 0 (Goal Seeking)  — drive toward goal until obstacle detected
//   State 1 (Wall Following) — circumnavigate obstacle until goal bearing clear
void bugAlgorithm() {
    setServoAngle(90);
    delay(100);
    long frontDist = readDistanceCm();

    switch (bugState) {
        case BUG_STATE_GOAL_SEEK:
            if (frontDist > 0 && frontDist < OBSTACLE_DIST_CM) {
                // Obstacle detected — transition to wall-following state
                motorsStop();
                buzz(300);
                bugState = BUG_STATE_WALL_FOLLOW;
            } else {
                motorsForward(BASE_SPEED);
            }
            break;

        case BUG_STATE_WALL_FOLLOW:
            wallFollowing();
            // Re-evaluate: if path ahead is clear, resume goal seeking
            setServoAngle(90);
            delay(100);
            frontDist = readDistanceCm();
            if (frontDist == 0 || frontDist >= OBSTACLE_DIST_CM * BUG_CLEARANCE_MULTIPLIER) {
                bugState = BUG_STATE_GOAL_SEEK;
            }
            break;
    }
}

// ─── Arduino Entry Points ──────────────────────────────────────────────────

void setup() {
    Serial.begin(9600);

    pinMode(TRIG_PIN,   OUTPUT);
    pinMode(ECHO_PIN,   INPUT);
    headServo.attach(SERVO_PIN);
    pinMode(ENA,        OUTPUT);
    pinMode(IN1,        OUTPUT);
    pinMode(IN2,        OUTPUT);
    pinMode(ENB,        OUTPUT);
    pinMode(IN3,        OUTPUT);
    pinMode(IN4,        OUTPUT);
    pinMode(IR_LEFT,    INPUT);
    pinMode(IR_RIGHT,   INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    setServoAngle(90); // centre the sensor head
    delay(500);
    Serial.println(F("AMR initialised — entering main loop."));
}

void loop() {
    switch (MODE) {
        case MODE_OBSTACLE_AVOID:
            obstacleAvoidance();
            break;
        case MODE_LINE_FOLLOW:
            lineFollowing();
            break;
        case MODE_WALL_FOLLOW:
            wallFollowing();
            break;
        case MODE_BUG_ALGORITHM:
            bugAlgorithm();
            break;
        default:
            motorsStop();
            break;
    }
}
