/*
 * MAIN ROBOT FIRMWARE
 * Environmental monitoring rover with sensors and navigation
 */

#include <Arduino.h>
#include "lora_link.h"
#include "protocol.h"
#include "app_config.h"
#include "main_config.h"
#include "sensors.h"
#include "navigation.h"

// Forward declarations
void handleLoRaPacket(LoRaPacket* packet);
void collectSensorData();
void transmitData();
void syncWithSubRobot();
void printStatus();

// LoRa communication
LoRaLink loraLink;

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long lastTransmitTime = 0;
unsigned long lastSyncTime = 0;
unsigned long lastStatusTime = 0;

// Data buffer
String dataBuffer[10];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("====================================");
  Serial.println("   MAIN ROBOT - Environmental Rover");
  Serial.println("====================================");
  
  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  
  // Initialize LoRa
  if (!loraLink.begin(LORA_FREQUENCY, LORA_CS, LORA_RST, LORA_DIO0)) {
    Serial.println("FATAL: LoRa initialization failed!");
    while (1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(200);
    }
  }
  
  loraLink.setConfiguration(LORA_TX_POWER, LORA_SPREADING_FACTOR, LORA_SYNC_WORD);
  
  // Initialize sensors
  initSensors();
  
  // Initialize navigation
  initNavigation();
  
  // TODO: Initialize WiFi/MQTT
  Serial.println("WiFi/MQTT: Not initialized (TODO)");
  
  Serial.println("Main Robot Ready!");
  Serial.println("====================================");
  
  digitalWrite(STATUS_LED, LOW);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle incoming LoRa packets
  LoRaPacket packet;
  if (loraLink.receivePacket(&packet)) {
    handleLoRaPacket(&packet);
  }
  
  // Update navigation (obstacle avoidance, GPS)
  updateNavigation();
  
  // Check battery and return home if needed
  checkReturnHome();
  
  // Collect sensor data periodically
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    collectSensorData();
  }
  
  // Transmit data periodically
  if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL) {
    lastTransmitTime = currentTime;
    transmitData();
  }
  
  // Sync with sub-robot periodically
  if (currentTime - lastSyncTime >= SYNC_INTERVAL) {
    lastSyncTime = currentTime;
    syncWithSubRobot();
  }
  
  // Print status periodically
  if (currentTime - lastStatusTime >= 5000) {
    lastStatusTime = currentTime;
    printStatus();
  }
  
  delay(10);
}

void handleLoRaPacket(LoRaPacket* packet) {
  switch (packet->type) {
    case PKT_PONG:
      Serial.println("Received PONG from sub-robot");
      break;
      
    case PKT_STATUS:
      Serial.print("Sub-robot status: ");
      Serial.println(packet->data);
      break;
      
    case PKT_DATA:
      Serial.print("Sub-robot data: ");
      Serial.println(packet->data);
      // TODO: Forward to cloud
      break;
      
    case PKT_SYNC_RESPONSE:
      Serial.println("Sync response received");
      break;
      
    default:
      Serial.print("Unknown packet type: ");
      Serial.println(packet->type, HEX);
  }
}

void collectSensorData() {
  Serial.println("Collecting sensor data...");
  
  // Update all sensors
  updateSensors();
  
  // Get JSON data
  String jsonData = getSensorDataJSON();
  
  // Store in buffer
  dataBuffer[bufferIndex] = jsonData;
  bufferIndex = (bufferIndex + 1) % 10;
  
  Serial.println("Data collected:");
  Serial.println(jsonData);
  
  // Send to sub-robot for backup
  loraLink.sendPacket(PKT_DATA, jsonData.c_str());
}

void transmitData() {
  Serial.println("Transmitting data...");
  
  // TODO: Send buffered data via MQTT if WiFi connected
  // if (isWiFiConnected()) {
  //   for (int i = 0; i < 10; i++) {
  //     if (dataBuffer[i].length() > 0) {
  //       publishMQTT(dataBuffer[i]);
  //     }
  //   }
  // }
  
  Serial.println("WiFi/MQTT not implemented yet");
}

void syncWithSubRobot() {
  Serial.println("Syncing with sub-robot...");
  
  // Send sync request
  loraLink.sendPacket(PKT_SYNC_REQUEST, "SYNC");
  
  // Send status request
  delay(100);
  loraLink.sendPacket(PKT_COMMAND, CMD_GET_STATUS);
}

void printStatus() {
  SensorData data = getSensorData();
  
  Serial.println("\n===== ROVER STATUS =====");
  Serial.print("Battery: ");
  Serial.print(data.batteryPercent, 1);
  Serial.print("% (");
  Serial.print(data.batteryVoltage, 2);
  Serial.println("V)");
  
  Serial.print("GPS: ");
  if (data.gpsValid) {
    Serial.print(data.latitude, 6);
    Serial.print(", ");
    Serial.println(data.longitude, 6);
  } else {
    Serial.println("No fix");
  }
  
  Serial.print("Air: ");
  Serial.print(data.airTemp, 1);
  Serial.print("°C, ");
  Serial.print(data.airHumidity, 0);
  Serial.println("%");
  
  Serial.print("Obstacle: ");
  Serial.print(getFrontDistance(), 0);
  Serial.println(" cm");
  
  Serial.print("LoRa RSSI: ");
  Serial.print(loraLink.getRSSI());
  Serial.println(" dBm");
  Serial.println("========================\n");
}
