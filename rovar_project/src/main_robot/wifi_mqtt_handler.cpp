#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "wifi_mqtt_handler.h"
#include "motor_control.h"

// Track if remote ESP is connected
volatile bool remoteConnected = false;
unsigned long lastRemotePacketTime = 0;
#define REMOTE_TIMEOUT 5000  // 5 seconds

// Shared motor command
extern volatile MotorCommand currentMotor;

bool initESPNowReceiver() {
  Serial.println("[ESP-NOW] Initializing receiver...");
  
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Initialization failed!");
    return false;
  }
  
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(MotorCommand)) {
      memcpy((void *)&currentMotor, incomingData, sizeof(MotorCommand));
      lastRemotePacketTime = millis();
      remoteConnected = true;
    }
  });
  
  Serial.println("[ESP-NOW] Receiver ready");
  return true;
}

bool isRemoteConnected() {
  // Check if we've received data within the timeout window
  if (millis() - lastRemotePacketTime > REMOTE_TIMEOUT) {
    remoteConnected = false;
  }
  return remoteConnected;
}

