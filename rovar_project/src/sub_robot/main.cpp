/*
 * SUB ROBOT FIRMWARE
 * WiFi relay and data buffer for extended range
 */

#include <Arduino.h>
#include <LoRa.h>
#include "subrobot_lora_init.h"
#include "protocol.h"
#include "app_config.h"
#include "esp_now_receiver.h"
#include "sd_card_handler.h"

// Forward declarations
void handleLoRaPacket(LoRaPacket* packet);
void handleCommand(const char* cmd);
void reportStatus();
bool sendLoRaPacket(uint8_t type, const char* data);
bool receiveLoRaPacket(LoRaPacket* packet);
void printJoystick(const JoystickData& data);

// LoRa communication
uint16_t loraPacketCounter = 0;
uint16_t loraCounter = 0;

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
  
  initEspNowReceiver();

  // Initialize LoRa
  if (!initSubRobotLoRa()) {
    Serial.println("FATAL: LoRa initialization failed!");
    while (1) {
      delay(1000);
    }
  }
  subRobotLoRaStartReceive();
  Serial.println("[LoRa] Ready to receive sensor data from main robot");
  
  // Initialize SD card for data logging
  Serial.println("Initializing SD card...");
  if (!initSDCard()) {
    Serial.println("WARNING: SD card not available - logging disabled");
  }
  
  // TODO: Initialize WiFi AP (optional relay mode)
  Serial.println("WiFi relay mode: disabled by default");
  // initRelayMode();
  
  Serial.println("Sub Robot Ready!");
  Serial.println("====================================\n");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle incoming LoRa packets from main robot
  LoRaPacket packet;
  if (receiveLoRaPacket(&packet)) {
    Serial.println("[LoRa RX] Packet received!");
    handleLoRaPacket(&packet);
  } else {
    // Debug: Show if module is available but not receiving
    if (subRobotLoRaAvailable()) {
      Serial.println("[LoRa] Available but parse failed");
    }
  }
  
  // Report status periodically
  if (currentTime - lastStatusTime >= 10000) {
    lastStatusTime = currentTime;
    reportStatus();
  }

  JoystickData joyData;
  if (espNowReadJoystick(joyData)) {
    printJoystick(joyData);
  }
  
  delay(10);
}

void handleLoRaPacket(LoRaPacket* packet) {
  switch (packet->type) {
    case PKT_PING:
      Serial.println("[Handler] Received PING, sending PONG");
      sendLoRaPacket(PKT_PONG, "OK");
      break;
      
    case PKT_COMMAND:
      Serial.print("[Handler] Command received: ");
      Serial.println(packet->data);
      handleCommand(packet->data);
      break;
      
    case PKT_DATA:
      Serial.println("========== SENSOR DATA RECEIVED ==========");
      Serial.print("Data: ");
      
      // Store to SD card
      if (storeLoRaData(packet->data)) {
        Serial.println("[Storage] Data logged to SD card");
      }
      
      Serial.println(packet->data);
      Serial.println("==========================================");
      storedDataCount++;
      sendLoRaPacket(PKT_ACK, "DATA_RECEIVED");
      break;
      
    case PKT_SYNC_REQUEST:
      Serial.println("[Handler] Sync request received");
      sendLoRaPacket(PKT_SYNC_RESPONSE, "READY");
      break;
      
    default:
      Serial.print("[Handler] Unknown packet type 0x");
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
    sendLoRaPacket(PKT_STATUS, response);
  }
  else if (strcmp(cmd, CMD_ENABLE_RELAY) == 0) {
    relayModeEnabled = true;
    sendLoRaPacket(PKT_ACK, "RELAY_ENABLED");
    Serial.println("Relay mode enabled");
  }
  else if (strcmp(cmd, CMD_DISABLE_RELAY) == 0) {
    relayModeEnabled = false;
    sendLoRaPacket(PKT_ACK, "RELAY_DISABLED");
    Serial.println("Relay mode disabled");
  }
  else if (strcmp(cmd, CMD_CLEAR_BUFFER) == 0) {
    storedDataCount = 0;
    sendLoRaPacket(PKT_ACK, "BUFFER_CLEARED");
    Serial.println("Buffer cleared");
  }
  else {
    sendLoRaPacket(PKT_ACK, "UNKNOWN_COMMAND");
  }
}

void reportStatus() {
  int currentRSSI = subRobotLoRaGetRSSI();
  
  char sdStatus[100];
  getSDCardStatus(sdStatus, sizeof(sdStatus));
  
  char status[200];
  snprintf(status, sizeof(status), 
           "MODE:%s,DATA:%d,RSSI:%d,%s", 
           relayModeEnabled ? "RELAY" : "BUFFER",
           storedDataCount,
           currentRSSI,
           sdStatus);
  
  sendLoRaPacket(PKT_STATUS, status);
  
  Serial.print("[Status Report] ");
  Serial.print(status);
  Serial.print(" | Listening...");
  Serial.println();
}

void printJoystick(const JoystickData& data) {
  Serial.print("X: ");
  Serial.print(data.x);
  Serial.print(" | Y: ");
  Serial.print(data.y);
  Serial.print(" | BTN: ");
  Serial.println(data.button);
}

bool sendLoRaPacket(uint8_t type, const char* data) {
  uint8_t buffer[4 + MAX_DATA_SIZE];
  size_t dataLen = strlen(data);
  if (dataLen > MAX_DATA_SIZE) {
    dataLen = MAX_DATA_SIZE;
  }

  buffer[0] = type;
  buffer[1] = (uint8_t)(loraPacketCounter >> 8);
  buffer[2] = (uint8_t)(loraPacketCounter & 0xFF);
  buffer[3] = (uint8_t)dataLen;
  memcpy(&buffer[4], data, dataLen);

  int state = subRobotLoRaTransmitPacket(buffer, 4 + dataLen);
  subRobotLoRaStartReceive();
  if (!state) {
    Serial.print("LoRa TX failed");
    return false;
  }

  loraPacketCounter++;
  return true;
}

bool receiveLoRaPacket(LoRaPacket* packet) {
  if (!subRobotLoRaAvailable()) {
    return false;
  }

  Serial.println("[LoRa] Module available, reading...");
  
  uint8_t buffer[MAX_PACKET_SIZE];
  int state = subRobotLoRaRead(buffer, sizeof(buffer));
  subRobotLoRaStartReceive();

  // LoRa library doesn't use state codes like RadioLib

  // Debug: Show received bytes
  Serial.print("[LoRa] Raw data received: ");
  for (int i = 0; i < 8 && i < sizeof(buffer); i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println("...");

  uint8_t dataLen = buffer[3];
  if (dataLen > MAX_DATA_SIZE) {
    Serial.print("[LoRa] Invalid data length: ");
    Serial.println(dataLen);
    return false;
  }

  packet->type = buffer[0];
  packet->counter = (uint16_t)((buffer[1] << 8) | buffer[2]);
  packet->dataLen = dataLen;
  memcpy(packet->data, &buffer[4], dataLen);
  packet->data[dataLen] = '\0';
  
  Serial.print("[LoRa] Parsed - Type: 0x");
  Serial.print(packet->type, HEX);
  Serial.print(" | Counter: ");
  Serial.print(packet->counter);
  Serial.print(" | DataLen: ");
  Serial.print(packet->dataLen);
  Serial.print(" | Data: ");
  Serial.println(packet->data);
  
  return true;
}
