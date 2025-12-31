/*
 * Navigation Manager Implementation
 */

#include "navigation.h"
#include "main_config.h"
#include "sensors.h"

// Motor state
bool motorsEnabled = true;
bool returningHome = false;

// Obstacle distances
float frontDistance = 999.0;
float leftDistance = 999.0;
float rightDistance = 999.0;

// Measure ultrasonic distance
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  return duration * 0.034 / 2.0;
}

void initNavigation() {
  Serial.println("Initializing navigation...");
  
  // Motor pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  
  // Ultrasonic pins
  pinMode(ULTRASONIC_FRONT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_FRONT_ECHO, INPUT);
  pinMode(ULTRASONIC_LEFT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO, INPUT);
  pinMode(ULTRASONIC_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_RIGHT_ECHO, INPUT);
  
  stopMotors();
  Serial.println("Navigation initialized");
}

void updateNavigation() {
  // Update obstacle sensors
  frontDistance = measureDistance(ULTRASONIC_FRONT_TRIG, ULTRASONIC_FRONT_ECHO);
  leftDistance = measureDistance(ULTRASONIC_LEFT_TRIG, ULTRASONIC_LEFT_ECHO);
  rightDistance = measureDistance(ULTRASONIC_RIGHT_TRIG, ULTRASONIC_RIGHT_ECHO);
  
  // Auto obstacle avoidance
  if (frontDistance < OBSTACLE_THRESHOLD && motorsEnabled) {
    handleObstacle();
  }
}

void moveForward(int speed) {
  if (!motorsEnabled) return;
  speed = constrain(speed, 0, 255);
  
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}

void moveBackward(int speed) {
  if (!motorsEnabled) return;
  speed = constrain(speed, 0, 255);
  
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}

void turnLeft(int speed) {
  if (!motorsEnabled) return;
  speed = constrain(speed, 0, 255);
  
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}

void turnRight(int speed) {
  if (!motorsEnabled) return;
  speed = constrain(speed, 0, 255);
  
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}

void setMotorsEnabled(bool enabled) {
  motorsEnabled = enabled;
  if (!enabled) stopMotors();
}

bool isPathClear() {
  return frontDistance > OBSTACLE_THRESHOLD;
}

float getFrontDistance() {
  return frontDistance;
}

void handleObstacle() {
  Serial.println("Obstacle detected! Avoiding...");
  
  stopMotors();
  delay(200);
  
  moveBackward(MOTOR_SPEED_SLOW);
  delay(500);
  stopMotors();
  delay(200);
  
  if (leftDistance > rightDistance) {
    turnLeft(MOTOR_SPEED_SLOW);
  } else {
    turnRight(MOTOR_SPEED_SLOW);
  }
  delay(500);
  stopMotors();
}

bool isGPSValid() {
  SensorData data = getSensorData();
  return data.gpsValid;
}

void getGPSPosition(float* lat, float* lon) {
  SensorData data = getSensorData();
  *lat = data.latitude;
  *lon = data.longitude;
}

float getDistanceToHome() {
  if (!isGPSValid()) return 0.0;
  
  float lat, lon;
  getGPSPosition(&lat, &lon);
  
  // Haversine formula (simplified)
  float dLat = (HOME_LATITUDE - lat) * 111000.0;
  float dLon = (HOME_LONGITUDE - lon) * 111000.0 * cos(lat * 0.017453);
  
  return sqrt(dLat * dLat + dLon * dLon);
}

void checkReturnHome() {
  SensorData data = getSensorData();
  
  if (data.batteryPercent < LOW_BATTERY_PERCENT && !returningHome) {
    startReturnHome();
  }
}

void startReturnHome() {
  Serial.println("Starting return home sequence...");
  returningHome = true;
  // TODO: Implement navigation to home coordinates
}

bool isReturningHome() {
  return returningHome;
}
