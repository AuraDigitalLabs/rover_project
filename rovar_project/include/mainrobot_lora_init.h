/*
 * Main Robot LoRa Initialization (LoRa Library)
 */

#ifndef MAIN_ROBOT_LORA_INIT_H
#define MAIN_ROBOT_LORA_INIT_H

#include <Arduino.h>

extern uint16_t loraCounter;

bool initMainRobotLoRa();
bool sendLoRaPacket(uint8_t type, const String& data);
int mainRobotLoRaTransmit(const String& msg);
int mainRobotLoRaTransmitPacket(const uint8_t* data, size_t len);
void mainRobotLoRaStartReceive();
bool mainRobotLoRaAvailable();
int mainRobotLoRaRead(uint8_t* out, size_t len);
int mainRobotLoRaGetRSSI();

#endif // MAIN_ROBOT_LORA_INIT_H
