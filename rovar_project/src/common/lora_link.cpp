/*
 * LoRa Link Implementation
 * Shared communication layer for both robots
 */

#include <LoRa.h>
#include <SPI.h>
#include "lora_link.h"
#include "app_config.h"

LoRaLink::LoRaLink() : _packetCounter(0), _initialized(false) {
}

bool LoRaLink::begin(long frequency, int8_t cs, int8_t rst, int8_t dio0) {
  Serial.println("Initializing LoRa...");
  
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, cs);
  LoRa.setPins(cs, rst, dio0);
  
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa initialization failed!");
    return false;
  }
  
  _initialized = true;
  Serial.println("LoRa initialized successfully!");
  return true;
}

void LoRaLink::setConfiguration(int txPower, int spreadingFactor, uint8_t syncWord) {
  if (!_initialized) return;
  
  LoRa.setTxPower(txPower);
  LoRa.setSpreadingFactor(spreadingFactor);
  LoRa.setSyncWord(syncWord);
  LoRa.enableCrc();
  
  Serial.print("LoRa configured - SF: ");
  Serial.print(spreadingFactor);
  Serial.print(", TX Power: ");
  Serial.print(txPower);
  Serial.println(" dBm");
}

bool LoRaLink::sendPacket(uint8_t type, const char* data) {
  if (!_initialized) return false;
  
  LoRaPacket packet;
  packet.type = type;
  packet.counter = _packetCounter++;
  
  // Copy data
  size_t dataLen = strlen(data);
  if (dataLen > MAX_DATA_SIZE) {
    dataLen = MAX_DATA_SIZE;
    Serial.println("Warning: Data truncated to fit packet");
  }
  
  packet.dataLen = dataLen;
  memcpy(packet.data, data, dataLen);
  packet.data[dataLen] = '\0';
  
  // Send packet
  LoRa.beginPacket();
  LoRa.write(packet.type);
  LoRa.write((uint8_t)(packet.counter >> 8));
  LoRa.write((uint8_t)(packet.counter & 0xFF));
  LoRa.write(packet.dataLen);
  LoRa.write((uint8_t*)packet.data, packet.dataLen);
  LoRa.endPacket();
  
  Serial.print("TX [");
  Serial.print(packet.type, HEX);
  Serial.print("] #");
  Serial.print(packet.counter);
  Serial.print(": ");
  Serial.println(packet.data);
  
  return true;
}

bool LoRaLink::receivePacket(LoRaPacket* packet) {
  if (!_initialized) return false;
  
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    return false;
  }
  
  // Read packet
  packet->type = LoRa.read();
  packet->counter = (LoRa.read() << 8) | LoRa.read();
  packet->dataLen = LoRa.read();
  
  // Read data
  int idx = 0;
  while (LoRa.available() && idx < packet->dataLen) {
    packet->data[idx++] = LoRa.read();
  }
  packet->data[idx] = '\0';
  
  // Get signal quality
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  Serial.print("RX [");
  Serial.print(packet->type, HEX);
  Serial.print("] #");
  Serial.print(packet->counter);
  Serial.print(" RSSI:");
  Serial.print(rssi);
  Serial.print(" SNR:");
  Serial.print(snr);
  Serial.print(": ");
  Serial.println(packet->data);
  
  return true;
}

int LoRaLink::getRSSI() {
  return LoRa.packetRssi();
}

float LoRaLink::getSNR() {
  return LoRa.packetSnr();
}

bool LoRaLink::isHealthy() {
  return _initialized;
}

uint16_t LoRaLink::getPacketCounter() {
  return _packetCounter;
}
