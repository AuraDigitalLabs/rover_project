#include <Arduino.h>
#include "motor_control.h"

// Motor state tracking
volatile MotorCommand currentMotor = {0, 0};

void initMotors() {
  Serial.println("[Motors] Initializing 6-motor rover system...");
  
  // Direction pins
  pinMode(LEFT_DIR1, OUTPUT);
  pinMode(LEFT_DIR2, OUTPUT);
  pinMode(RIGHT_DIR1, OUTPUT);
  pinMode(RIGHT_DIR2, OUTPUT);
  
  // Speed pins (PWM)
  ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_SPEED, LEFT_PWM_CH);
  ledcAttachPin(RIGHT_SPEED, RIGHT_PWM_CH);
  
  stopMotors();
  Serial.println("[Motors] 6-motor rover initialized successfully");
  Serial.println("[Motors] Left:  DIR1=GPIO13, DIR2=GPIO33, PWM=GPIO25");
  Serial.println("[Motors] Right: DIR1=GPIO12, DIR2=GPIO15, PWM=GPIO4");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Constrain values
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Left motor control (all 3 left motors together)
  if (leftSpeed > 0) {
    // Forward: DIR1=HIGH, DIR2=LOW
    digitalWrite(LEFT_DIR1, HIGH);
    digitalWrite(LEFT_DIR2, LOW);
    ledcWrite(LEFT_PWM_CH, leftSpeed);
  } else if (leftSpeed < 0) {
    // Backward: DIR1=LOW, DIR2=HIGH
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, HIGH);
    ledcWrite(LEFT_PWM_CH, -leftSpeed);
  } else {
    // Stop: both LOW
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, LOW);
    ledcWrite(LEFT_PWM_CH, 0);
  }
  
  // Right motor control (all 3 right motors together)
  if (rightSpeed > 0) {
    // Forward: DIR1=HIGH, DIR2=LOW
    digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, LOW);
    ledcWrite(RIGHT_PWM_CH, rightSpeed);
  } else if (rightSpeed < 0) {
    // Backward: DIR1=LOW, DIR2=HIGH
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, HIGH);
    ledcWrite(RIGHT_PWM_CH, -rightSpeed);
  } else {
    // Stop: both LOW
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, LOW);
    ledcWrite(RIGHT_PWM_CH, 0);
  }
  
  currentMotor.leftSpeed = leftSpeed;
  currentMotor.rightSpeed = rightSpeed;
}

void stopMotors() {
  digitalWrite(LEFT_DIR1, LOW);
  digitalWrite(LEFT_DIR2, LOW);
  digitalWrite(RIGHT_DIR1, LOW);
  digitalWrite(RIGHT_DIR2, LOW);
  ledcWrite(LEFT_PWM_CH, 0);
  ledcWrite(RIGHT_PWM_CH, 0);
  currentMotor.leftSpeed = 0;
  currentMotor.rightSpeed = 0;
}

void motorsEmergencyStop() {
  stopMotors();
  Serial.println("[Motors] EMERGENCY STOP ACTIVATED");
}

void handleMotorControl(const MotorCommand& cmd) {
  setMotorSpeed(cmd.leftSpeed, cmd.rightSpeed);
}

void handleJoystickInput(const JoystickData& joy) {
  // Convert joystick analog values (0-4095) to motor speeds (-255 to +255)
  // X: left-right movement (left turn, right turn, straight)
  // Y: forward-backward movement (forward, backward, stop)
  
  // Apply deadzone
  int joyX = joy.x;
  int joyY = joy.y;
  
  if (abs(joyX - JOYSTICK_CENTER_X) < JOYSTICK_DEADZONE) {
    joyX = JOYSTICK_CENTER_X;  // Center X for straight movement
  }
  
  if (abs(joyY - JOYSTICK_CENTER_Y) < JOYSTICK_DEADZONE) {
    joyY = JOYSTICK_CENTER_Y;  // Center Y for no forward/backward
  }
  
  // Calculate normalized values (-1.0 to +1.0)
  float forwardRaw = (JOYSTICK_CENTER_Y - joyY) / (float)JOYSTICK_CENTER_Y;  // Y-axis (forward/backward)
  float turnRaw = (joyX - JOYSTICK_CENTER_X) / (float)JOYSTICK_CENTER_X;      // X-axis (left/right turn)
  
  // Constrain to -1.0 to +1.0
  forwardRaw = constrain(forwardRaw, -1.0f, 1.0f);
  turnRaw = constrain(turnRaw, -1.0f, 1.0f);
  
  // Calculate differential drive
  // Forward/Backward is priority; turning adjusts both sides
  float leftMotor = (forwardRaw - turnRaw) * 127.5f;   // Range: -255 to +255
  float rightMotor = (forwardRaw + turnRaw) * 127.5f;  // Range: -255 to +255
  
  // Constrain final values
  int leftSpeed = constrain((int)leftMotor, -255, 255);
  int rightSpeed = constrain((int)rightMotor, -255, 255);
  
  // Emergency stop if button pressed
  if (joy.button) {
    motorsEmergencyStop();
    return;
  }
  
  // Set motor speeds
  setMotorSpeed(leftSpeed, rightSpeed);
  
  // Debug output (optional)
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint > 200) {  // Print every 200ms to avoid spam
    lastDebugPrint = millis();
    Serial.print("[Joystick] X:");
    Serial.print(joyX);
    Serial.print(" Y:");
    Serial.print(joyY);
    Serial.print(" | Left:");
    Serial.print(leftSpeed);
    Serial.print(" Right:");
    Serial.print(rightSpeed);
    Serial.print(" | BTN:");
    Serial.println(joy.button);
  }
}
