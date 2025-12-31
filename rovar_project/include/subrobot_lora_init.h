/*
 * Sub Robot LoRa Initialization (LoRa Library)
 */

#ifndef SUB_ROBOT_LORA_INIT_H
#define SUB_ROBOT_LORA_INIT_H

#include <Arduino.h>

extern uint16_t loraCounter;

bool initSubRobotLoRa();
void subRobotLoRaStartReceive();
bool subRobotLoRaAvailable();
int subRobotLoRaRead(String& out);
int subRobotLoRaRead(uint8_t* out, size_t len);
int subRobotLoRaTransmitPacket(const uint8_t* data, size_t len);
int subRobotLoRaGetRSSI();

#endif // SUB_ROBOT_LORA_INIT_H
