#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "esp_now_receiver.h"  // For JoystickData struct

// ========== 6-MOTOR ROVER CONFIGURATION ==========
// Left side motors (3 motors tied together)
#define LEFT_DIR1       13    // Direction control 1
#define LEFT_DIR2       33    // Direction control 2
#define LEFT_SPEED      25    // PWM speed control

// Right side motors (3 motors tied together)
#define RIGHT_DIR1      12    // Direction control 1 (strap pin, usable)
#define RIGHT_DIR2      15    // Direction control 2 (strap pin, usable)
#define RIGHT_SPEED     4     // PWM speed control

// PWM Configuration
#define LEFT_PWM_CH     0
#define RIGHT_PWM_CH    1
#define PWM_FREQ        5000           // 5 kHz PWM frequency
#define PWM_RESOLUTION  8              // 8-bit resolution (0-255)

// Joystick thresholds
#define JOYSTICK_CENTER_X   1900       // Center position (~1900 for 0-4095 range)
#define JOYSTICK_CENTER_Y   1900
#define JOYSTICK_DEADZONE   100        // Deadzone to prevent drift

// Motor command structure
struct MotorCommand {
  int leftSpeed;   // -255 to +255
  int rightSpeed;  // -255 to +255
};

// Function declarations
void initMotors();
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();
void handleMotorControl(const MotorCommand& cmd);
void handleJoystickInput(const JoystickData& joy);
void motorsEmergencyStop();

#endif // MOTOR_CONTROL_H
