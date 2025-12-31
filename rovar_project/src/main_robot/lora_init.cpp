/*
 * Main Robot LoRa Initialization (LoRa Library - Simpler)
 */

#include "mainrobot_lora_init.h"
#include "protocol.h"
#include <SPI.h>
#include <LoRa.h>

// RA-02 pins
#define LORA_CS     5
#define LORA_RST    14
#define LORA_DIO0   26

#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23

bool initMainRobotLoRa() {
  Serial.println("[LoRa] Starting LoRa transmitter...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

#ifdef LORA_FREQUENCY
  long freq = LORA_FREQUENCY;
#else
  long freq = 433E6;
#endif

  if (!LoRa.begin(freq)) {
    Serial.println("[LoRa] Init failed!");
    return false;
  }

  // Configure LoRa
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  
  Serial.print("[LoRa] Frequency: ");
  Serial.print(freq);
  Serial.println(" Hz");
  Serial.println("[LoRa] Init success!");
  return true;
}

bool sendLoRaPacket(uint8_t type, const String& data) {
  uint8_t buffer[MAX_PACKET_SIZE];
  int len = data.length();
  
  buffer[0] = type;
  buffer[1] = (loraCounter >> 8) & 0xFF;
  buffer[2] = loraCounter & 0xFF;
  buffer[3] = len;
  
  data.getBytes(buffer + 4, len + 1);
  
  LoRa.beginPacket();
  LoRa.write(buffer, 4 + len);
  int result = LoRa.endPacket();
  
  Serial.print("[LoRa TX] ");
  if (result) {
    Serial.print("SUCCESS | Type: 0x");
    Serial.print(type, HEX);
    Serial.print(" | Data: ");
    Serial.println(data);
  } else {
    Serial.println("FAILED");
  }
  
  loraCounter++;
  return result;
}

int mainRobotLoRaTransmit(const String& msg) {
  return 0;
}

int mainRobotLoRaTransmitPacket(const uint8_t* data, size_t len) {
  LoRa.beginPacket();
  LoRa.write(data, len);
  return LoRa.endPacket();
}

void mainRobotLoRaStartReceive() {
  LoRa.receive();
}

bool mainRobotLoRaAvailable() {
  int packetSize = LoRa.parsePacket();
  return packetSize > 0;
}

int mainRobotLoRaRead(uint8_t* out, size_t len) {
  size_t count = 0;
  while (LoRa.available() && count < len) {
    out[count++] = LoRa.read();
  }
  return 0;
}

int mainRobotLoRaGetRSSI() {
  return LoRa.packetRssi();
}
