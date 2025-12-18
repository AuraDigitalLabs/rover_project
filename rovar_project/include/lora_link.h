/*
 * LoRa Link - Shared Communication Layer
 * Used by both Main and Sub robots
 */

#ifndef LORA_LINK_H
#define LORA_LINK_H

#include <Arduino.h>
#include "protocol.h"

class LoRaLink {
public:
  LoRaLink();
  
  // Initialization
  bool begin(long frequency, int8_t cs, int8_t rst, int8_t dio0);
  void setConfiguration(int txPower, int spreadingFactor, uint8_t syncWord);
  
  // Send/Receive
  bool sendPacket(uint8_t type, const char* data);
  bool receivePacket(LoRaPacket* packet);
  
  // Status
  int getRSSI();
  float getSNR();
  bool isHealthy();
  
  // Utilities
  uint16_t getPacketCounter();
  
private:
  uint16_t _packetCounter;
  bool _initialized;
};

#endif // LORA_LINK_H
