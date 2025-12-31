/*
 * ESP-NOW joystick receiver
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_now_receiver.h"

static JoystickData rxData;
static volatile bool rxUpdated = false;

// -------- ESP-NOW receive callback --------
static void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  (void)len;
  memcpy(&rxData, data, sizeof(rxData));
  rxUpdated = true;
}

void initEspNowReceiver() {
  // WiFi in station mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < 10000) {
    delay(200);
  }
  Serial.print("WiFi status: ");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("connected");
  } else {
    Serial.println("not connected");
  }

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(onReceive);

  Serial.println("ESP-NOW Receiver Ready");
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
}

bool espNowReadJoystick(JoystickData& out) {
  if (!rxUpdated) {
    return false;
  }

  out = rxData;
  rxUpdated = false;
  return true;
}
