/*
 * Sub Robot LoRa Initialization (LoRa Library - Simpler)
 */

#include "subrobot_lora_init.h"
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

bool initSubRobotLoRa() {
  Serial.println("[LoRa] Starting LoRa receiver...");

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

void subRobotLoRaStartReceive() {
  LoRa.receive();
  Serial.println("[LoRa] RX mode active");
}

bool subRobotLoRaAvailable() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("[LoRa] Packet available! Size: ");
    Serial.print(packetSize);
    Serial.print(" bytes | RSSI: ");
    Serial.println(LoRa.packetRssi());
    return true;
  }
  return false;
}

int subRobotLoRaRead(String& out) {
  while (LoRa.available()) {
    out += (char)LoRa.read();
  }
  return 0;
}

int subRobotLoRaRead(uint8_t* out, size_t len) {
  size_t count = 0;
  while (LoRa.available() && count < len) {
    out[count++] = LoRa.read();
  }
  return 0;
}

int subRobotLoRaTransmitPacket(const uint8_t* data, size_t len) {
  LoRa.beginPacket();
  LoRa.write(data, len);
  return LoRa.endPacket();
}

int subRobotLoRaGetRSSI() {
  return LoRa.packetRssi();
}
