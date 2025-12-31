/*
 * SD Card Handler for Sub Robot
 * Stores incoming sensor data in CSV format
 */

#ifndef SD_CARD_HANDLER_H
#define SD_CARD_HANDLER_H

#include <Arduino.h>

// SD Card pins (using HSPI)
#define SD_CS   27
#define SD_SCK  16
#define SD_MISO 34
#define SD_MOSI 13

// Function declarations
bool initSDCard();
bool storeLoRaData(const char* sensorData);
void getSDCardStatus(char* statusBuffer, size_t bufferSize);
String getCurrentLogFileName();

#endif // SD_CARD_HANDLER_H
