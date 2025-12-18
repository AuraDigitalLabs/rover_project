/*
 * Navigation Manager for Main Robot
 * Handles GPS, motors, and obstacle avoidance
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

// Navigation functions
void initNavigation();
void updateNavigation();

// Motor control
void stopMotors();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void setMotorsEnabled(bool enabled);

// Obstacle avoidance
bool isPathClear();
float getFrontDistance();
void handleObstacle();

// GPS
bool isGPSValid();
void getGPSPosition(float* lat, float* lon);
float getDistanceToHome();

// Return home
void checkReturnHome();
void startReturnHome();
bool isReturningHome();

#endif // NAVIGATION_H
