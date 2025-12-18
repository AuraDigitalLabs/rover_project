/*
 * SUB ROBOT FIRMWARE
 * WiFi relay and data buffer for extended range
 */

#include <Arduino.h>
#include "lora_link.h"
#include "protocol.h"
#include "app_config.h"

// Forward declarations
void handleLoRaPacket(LoRaPacket* packet);
void handleCommand(const char* cmd);
void reportStatus();

// LoRa communication
LoRaLink loraLink;

// Status
unsigned long lastStatusTime = 0;
bool relayModeEnabled = false;
int storedDataCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("====================================");
  Serial.println("   SUB ROBOT - Relay & Buffer");
  Serial.println("====================================");
  
  // Initialize LoRa
  if (!loraLink.begin(LORA_FREQUENCY, LORA_CS, LORA_RST, LORA_DIO0)) {
    Serial.println("FATAL: LoRa initialization failed!");
    while (1) {
      delay(1000);
    }
  }
  
  loraLink.setConfiguration(LORA_TX_POWER, LORA_SPREADING_FACTOR, LORA_SYNC_WORD);
  
  // TODO: Initialize SD card
  Serial.println("Initializing SD card...");
  // initDataStorage();
  
  // TODO: Initialize WiFi AP (optional relay mode)
  Serial.println("WiFi relay mode: disabled by default");
  // initRelayMode();
  
  Serial.println("Sub Robot Ready!");
  Serial.println("====================================");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle incoming LoRa packets from main robot
  LoRaPacket packet;
  if (loraLink.receivePacket(&packet)) {
    handleLoRaPacket(&packet);
  }
  
  // Report status periodically
  if (currentTime - lastStatusTime >= 10000) {
    lastStatusTime = currentTime;
    reportStatus();
  }
  
  // TODO: Update relay mode if enabled
  // if (relayModeEnabled) {
  //   updateRelayMode();
  // }
  
  delay(10);
}

void handleLoRaPacket(LoRaPacket* packet) {
  switch (packet->type) {
    case PKT_PING:
      Serial.println("Received PING, sending PONG");
      loraLink.sendPacket(PKT_PONG, "OK");
      break;
      
    case PKT_COMMAND:
      handleCommand(packet->data);
      break;
      
    case PKT_DATA:
      Serial.print("Storing data: ");
      Serial.println(packet->data);
      // TODO: Store to SD card
      storedDataCount++;
      loraLink.sendPacket(PKT_ACK, "STORED");
      break;
      
    case PKT_SYNC_REQUEST:
      Serial.println("Sync request received");
      loraLink.sendPacket(PKT_SYNC_RESPONSE, "READY");
      break;
      
    default:
      Serial.print("Unknown packet type: ");
      Serial.println(packet->type, HEX);
  }
}

void handleCommand(const char* cmd) {
  Serial.print("Command: ");
  Serial.println(cmd);
  
  if (strcmp(cmd, CMD_GET_STATUS) == 0) {
    reportStatus();
  }
  else if (strcmp(cmd, CMD_GET_DATA_COUNT) == 0) {
    char response[50];
    snprintf(response, sizeof(response), "COUNT:%d", storedDataCount);
    loraLink.sendPacket(PKT_STATUS, response);
  }
  else if (strcmp(cmd, CMD_ENABLE_RELAY) == 0) {
    relayModeEnabled = true;
    loraLink.sendPacket(PKT_ACK, "RELAY_ENABLED");
    Serial.println("Relay mode enabled");
  }
  else if (strcmp(cmd, CMD_DISABLE_RELAY) == 0) {
    relayModeEnabled = false;
    loraLink.sendPacket(PKT_ACK, "RELAY_DISABLED");
    Serial.println("Relay mode disabled");
  }
  else if (strcmp(cmd, CMD_CLEAR_BUFFER) == 0) {
    storedDataCount = 0;
    loraLink.sendPacket(PKT_ACK, "BUFFER_CLEARED");
    Serial.println("Buffer cleared");
  }
  else {
    loraLink.sendPacket(PKT_ACK, "UNKNOWN_COMMAND");
  }
}

void reportStatus() {
  char status[100];
  snprintf(status, sizeof(status), 
           "MODE:%s,DATA:%d,RSSI:%d", 
           relayModeEnabled ? "RELAY" : "BUFFER",
           storedDataCount,
           loraLink.getRSSI());
  
  loraLink.sendPacket(PKT_STATUS, status);
  
  Serial.print("Status sent: ");
  Serial.println(status);
}
