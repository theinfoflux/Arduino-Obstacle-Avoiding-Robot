#include <AFMotor.h>
#include <Servo.h>

// Motors
AF_DCMotor motorLeft(4);
AF_DCMotor motorRight(1);

// Servo
Servo myServo;
const int servoPin = 10;

// Ultrasonic pins
const int trigPin = A4;
const int echoPin = A5;

// ------ Adjustable Settings ------
int motorSpeed = 130;           // Full speed
int minDistance = 50;           // Minimum safe stopping distance
int reverseTime = 250;          // Reverse duration before scanning
int scanDelay = 250;            // Delay after each servo sweep
int turnBaseTime = 150;         // Base turn duration
// ---------------------------------

// Globals
long duration;
int distance;

// Ultrasonic function
int getDistance() {
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH, 25000);
  distance = duration * 0.034 / 2;
  if (distance == 0) distance = 999;  // Sensor error fallback
  return distance;
}

// Motor helpers
void moveForward() {
  motorLeft.setSpeed(motorSpeed);
  motorRight.setSpeed(motorSpeed);
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
}

void reverseRobot(int timeMs) {
  motorLeft.setSpeed(motorSpeed);
  motorRight.setSpeed(motorSpeed);
  motorLeft.run(BACKWARD);
  motorRight.run(BACKWARD);
  delay(timeMs);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}

void stopRobot() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}

void turnLeftSmooth(int strength) {
  motorLeft.setSpeed(strength);
  motorRight.setSpeed(motorSpeed);
  motorLeft.run(BACKWARD);
  motorRight.run(FORWARD);
}

void turnRightSmooth(int strength) {
  motorLeft.setSpeed(motorSpeed);
  motorRight.setSpeed(strength);
  motorLeft.run(FORWARD);
  motorRight.run(BACKWARD);
}

// ------------ MULTI-ANGLE SCAN FUNCTION -------------
int scanAngles[5] = {20, 60, 90, 120, 160};
int scanResults[5];

void multiScan() {
  for (int i = 0; i < 5; i++) {
    myServo.write(scanAngles[i]);
    delay(scanDelay);
    scanResults[i] = getDistance();
    Serial.print("Angle "); Serial.print(scanAngles[i]);
    Serial.print(" => "); Serial.println(scanResults[i]);
  }
}
// -----------------------------------------------------

// --------- SELECT BEST DIRECTION FUNCTION ------------
int findBestDirection() {
  int bestIndex = 2; // default front position
  int bestDistance = scanResults[2];

  for (int i = 0; i < 5; i++) {
    if (scanResults[i] > bestDistance) {
      bestDistance = scanResults[i];
      bestIndex = i;
    }
  }
  return bestIndex;
}
// -----------------------------------------------------

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
}

void loop() {

  // Look forward
  myServo.write(90);
  delay(200);
  int frontDist = getDistance();
  Serial.print("Front Distance: ");
  Serial.println(frontDist);

  // CLEAR path → Move forward
  if (frontDist > minDistance) {
    moveForward();
    return;
  }

  // ----------- OBSTACLE DETECTED ------------
  stopRobot();
  delay(100);

  // Reverse a bit to avoid collision
  reverseRobot(reverseTime);

  // Perform multi-angle scan
  multiScan();

  // Select best direction
  int bestDirIndex = findBestDirection();
  int bestAngle = scanAngles[bestDirIndex];
  int bestDist = scanResults[bestDirIndex];

  Serial.print("Best direction angle: ");
  Serial.println(bestAngle);
  Serial.print("Best distance: ");
  Serial.println(bestDist);

  // If no safe direction → reverse again
  if (bestDist < minDistance + 10) {
    Serial.println("All sides blocked → reversing again");
    reverseRobot(500);
    return;
  }

  // ----------- ADVANCED TURN CALCULATION -----------
  int angleOffset = bestAngle - 90;    // how far from center
  int turnStrength = map(abs(angleOffset), 0, 70, 100, 180); // smooth turn power
  int turnDuration = turnBaseTime + abs(angleOffset) * 3;    // bigger angle = longer turn

  // Execute smooth turn
  if (angleOffset > 0) {
    Serial.println("Turning LEFT smoothly...");
    turnLeftSmooth(turnStrength);
  } else {
    Serial.println("Turning RIGHT smoothly...");
    turnRightSmooth(turnStrength);
  }

  delay(turnDuration);
  stopRobot();
  delay(150);

  // Re-scan front before moving again
  myServo.write(90);
  delay(200);
  frontDist = getDistance();

  if (frontDist > minDistance) {
    moveForward();
  }
}
