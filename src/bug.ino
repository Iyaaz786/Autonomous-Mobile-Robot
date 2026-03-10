/*
 * - MODE 0: Obstacle Avoidance
 * - MODE 1: Line Following
 * - MODE 2: Wall Following
 * - MODE 3: Combined Obstacle Avoidance + Line Following
 * - MODE 4: Bug Algorithm (Obstacle + Wall Following)
 */

#include <Servo.h>

// PIN DEFINITIONS

// Motor Driver
#define speedPinR 3
#define RightDirectPin1 12
#define RightDirectPin2 11
#define speedPinL 6
#define LeftDirectPin1 7
#define LeftDirectPin2 8

// Ultrasonic Sensor
#define Trig_PIN 10
#define Echo_PIN 2

// Servo
#define SERVO_PIN 9

// Buzzer
#define BUZZ_PIN 13

// 5 Channel IR Sensor Array
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4

#define MODE 4  // THIS CHANGES THE CURRENT MODE

// Speed Settings
#define NORMAL_SPEED 120
#define TURN_SPEED 150
#define SHARP_TURN_SPEED 180
#define BACK_SPEED 200

// Distance Thresholds (in cm)
#define OBSTACLE_STOP_DISTANCE 20
#define OBSTACLE_SLOW_DISTANCE 40
#define WALL_FOLLOW_DISTANCE 25
#define WALL_TOLERANCE 5

Servo head;
int frontDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

// MOTOR CONTROL FUNCTIONS
void stopMotors() {
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
}

void moveForward(int speed) {
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
  analogWrite(speedPinL, speed);
  analogWrite(speedPinR, speed);
}

void moveBackward(int speed) {
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
  analogWrite(speedPinR, speed);
  analogWrite(speedPinL, speed);
}

void turnLeft(int speed) {
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
  analogWrite(speedPinR, speed);
  analogWrite(speedPinL, speed);
}

void turnRight(int speed) {
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
  analogWrite(speedPinR, speed);
  analogWrite(speedPinL, speed);
}

void curveLeft(int leftSpeed, int rightSpeed) {
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
  analogWrite(speedPinL, leftSpeed);
  analogWrite(speedPinR, rightSpeed);
}

void curveRight(int leftSpeed, int rightSpeed) {
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
  analogWrite(speedPinL, leftSpeed);
  analogWrite(speedPinR, rightSpeed);
}

void buzz() {
  for (int i = 0; i < 50; i++) {
    digitalWrite(BUZZ_PIN, LOW);
    delay(1);
    digitalWrite(BUZZ_PIN, HIGH);
    delay(1);
  }
}

// ULTRASONIC SENSOR FUNCTION
int measureDistance() {
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN, LOW);

  long duration = pulseIn(Echo_PIN, HIGH, 38000);
  if (duration == 0) return 500; // Out of range

  int distance = duration * 0.034 / 2;
  return distance;
}

void scanEnvironment() {
  head.write(50);
  delay(300);
  frontDistance = measureDistance();

  head.write(30);
  delay(300);
  rightDistance = measureDistance();

  head.write(150);
  delay(300);
  leftDistance = measureDistance();

  head.write(90);
  delay(300);

  Serial.print("Distances - Front: ");
  Serial.print(frontDistance);
  Serial.print(" cm, Left: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right: ");
  Serial.print(rightDistance);
  Serial.println(" cm");
}

// MODE 0: OBSTACLE AVOIDANCE
void obstacleAvoidance() {
  frontDistance = measureDistance();

  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  if (frontDistance < OBSTACLE_STOP_DISTANCE) {
    stopMotors();
    buzz();
    Serial.println("Obstacle detected! Scanning...");

    moveBackward(BACK_SPEED);
    delay(400);
    stopMotors();

    scanEnvironment();

    // Decide which way to turn
    if (leftDistance > rightDistance && leftDistance > OBSTACLE_STOP_DISTANCE) {
      Serial.println("Turning LEFT");
      turnLeft(TURN_SPEED);
      delay(500);
    } else if (rightDistance > leftDistance && rightDistance > OBSTACLE_STOP_DISTANCE) {
      Serial.println("Turning RIGHT");
      turnRight(TURN_SPEED);
      delay(500);
    } else {
      // Both sides blocked - turn around
      Serial.println("Both sides blocked - U-TURN");
      turnRight(TURN_SPEED);
      delay(1000);
    }
    stopMotors();
    delay(100);
  } else if (frontDistance < OBSTACLE_SLOW_DISTANCE) {
    Serial.println("Slowing down - obstacle ahead");
    moveForward(NORMAL_SPEED / 2);
  } else {
    Serial.println("Path clear - moving forward");
    moveForward(NORMAL_SPEED);
  }
}

// MODE 1: LINE FOLLOWING
void lineFollowing() {
  int s1 = digitalRead(ir1);
  int s2 = digitalRead(ir2);
  int s3 = digitalRead(ir3);
  int s4 = digitalRead(ir4);
  int s5 = digitalRead(ir5);

  Serial.print("Sensors: ");
  Serial.print(s1); Serial.print(s2); Serial.print(s3);
  Serial.print(s4); Serial.println(s5);

  if (s3 == 0 && s2 == 1 && s4 == 1) {
    moveForward(NORMAL_SPEED);
    Serial.println("Forward");
  }
  else if (s2 == 0) {
    curveLeft(NORMAL_SPEED - 30, NORMAL_SPEED);
    Serial.println("Slight left");
  }
  else if (s1 == 0) {
    turnLeft(SHARP_TURN_SPEED);
    Serial.println("Sharp left");
  }
  else if (s4 == 0) {
    curveRight(NORMAL_SPEED, NORMAL_SPEED - 30);
    Serial.println("Slight right");
  }
  else if (s5 == 0) {
    turnRight(SHARP_TURN_SPEED);
    Serial.println("Sharp right");
  }
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1) {
    stopMotors();
    Serial.println("Line lost");
  }
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
    moveForward(NORMAL_SPEED);
    Serial.println("Intersection - continuing");
  }
}

// MODE 2: WALL FOLLOWING
void wallFollowing() {
  head.write(90);
  delay(250);
  frontDistance = measureDistance();

  head.write(30);
  delay(300);
  int wallDist = measureDistance();

  head.write(90);
  delay(200);

  Serial.print("Wall Right: ");
  Serial.print(wallDist);
  Serial.print(" cm, Front: ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  // PRIORITY 1: Front obstacle
  if (frontDistance < 25) {
    stopMotors();
    buzz();
    Serial.println("WALL AHEAD - Turning left 90°");

    moveBackward(BACK_SPEED);
    delay(300);
    stopMotors();
    delay(100);

    turnLeft(TURN_SPEED);
    delay(600);
    stopMotors();
    delay(200);
  }
  // PRIORITY 2: Too close to wall
  else if (wallDist < 15) {
    Serial.println("TOO CLOSE to wall - Turning left");
    turnLeft(TURN_SPEED);
    delay(200);
  }
  else if (wallDist < (WALL_FOLLOW_DISTANCE - WALL_TOLERANCE)) {
    Serial.println("Adjusting LEFT (closer than desired)");
    curveLeft(68, NORMAL_SPEED);
  }
  else if ((wallDist > (WALL_FOLLOW_DISTANCE + WALL_TOLERANCE)) && wallDist < 100) {
    Serial.println("Adjusting RIGHT (too far from wall)");
    curveRight(NORMAL_SPEED, 50);
  }
  else if (wallDist > 100) {
    Serial.println("Lost wall - searching right");
    curveRight(NORMAL_SPEED, 40);
  }
  else {
    Serial.println("Good distance - following wall");
    moveForward(NORMAL_SPEED);
  }
}

// MODE 3: COMBINED (Obstacle + Line)
void combinedMode() {
  frontDistance = measureDistance();

  if (frontDistance < OBSTACLE_STOP_DISTANCE) {
    Serial.println("OBSTACLE OVERRIDE");
    stopMotors();
    buzz();
    moveBackward(BACK_SPEED);
    delay(400);
    stopMotors();

    scanEnvironment();

    if (leftDistance > rightDistance) {
      turnLeft(TURN_SPEED);
      delay(500);
    } else {
      turnRight(TURN_SPEED);
      delay(500);
    }
    stopMotors();
  } else {
    // No obstacle - follow line
    lineFollowing();
  }
}

// MODE 4: BUG ALGORITHM
int bugState = 0; // 0=goal seeking, 1=wall following
int wallFollowTimer = 0;

void bugAlgorithm() {
  frontDistance = measureDistance();

  if (bugState == 0) {
    Serial.println("Moving to goal");

    if (frontDistance < OBSTACLE_STOP_DISTANCE) {
      Serial.println("Obstacle hit - switching to wall follow");
      bugState = 1;
      wallFollowTimer = 0;
      buzz();
      stopMotors();
      delay(200);
    } else {
      moveForward(NORMAL_SPEED);
    }
  } else {
    Serial.print("Following wall (");
    Serial.print(wallFollowTimer);
    Serial.println(")");

    wallFollowing();
    wallFollowTimer++;

    if (wallFollowTimer > 30 && frontDistance > OBSTACLE_SLOW_DISTANCE) {
      Serial.println("Bug: Path clear - returning to goal");
      bugState = 0;
      head.write(90);
    }
  }
}

// SETUP
void setup() {
  // Motor
  pinMode(RightDirectPin1, OUTPUT);
  pinMode(RightDirectPin2, OUTPUT);
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  // Sensor
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  digitalWrite(BUZZ_PIN, HIGH);

  // Servo
  head.attach(SERVO_PIN);
  head.write(90);

  Serial.begin(9600);
  Serial.println("=== ROBOT CONTROL SYSTEM ===");
  Serial.println("MODE 0: Obstacle Avoidance");
  Serial.println("MODE 1: Line Following");
  Serial.println("MODE 2: Wall Following");
  Serial.println("MODE 3: Combined (Obstacle + Line)");
  Serial.println("MODE 4: Bug Algorithm");
  Serial.print("CURRENT MODE: ");
  Serial.println(MODE);
  Serial.println("============================");

  delay(2000);
  buzz();
}

// MAIN LOOP
void loop() {
  switch (MODE) {
    case 0:
      obstacleAvoidance();
      delay(100);
      break;

    case 1:
      lineFollowing();
      delay(50);
      break;

    case 2:
      wallFollowing();
      delay(100);
      break;

    case 3:
      combinedMode();
      delay(50);
      break;

    case 4:
      bugAlgorithm();
      delay(100);
      break;

    default:
      stopMotors();
      Serial.println("Invalid MODE");
      delay(1000);
  }
}
