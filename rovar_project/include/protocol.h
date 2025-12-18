/*
 * Communication Protocol
 * Shared between Main Robot and Sub Robot
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// Packet types
#define PKT_PING            0x01
#define PKT_PONG            0x02
#define PKT_DATA            0x10
#define PKT_STATUS          0x11
#define PKT_COMMAND         0x20
#define PKT_ACK             0x30
#define PKT_SYNC_REQUEST    0x40
#define PKT_SYNC_RESPONSE   0x41

// Commands
#define CMD_GET_STATUS      "GET_STATUS"
#define CMD_GET_DATA_COUNT  "GET_DATA_COUNT"
#define CMD_SEND_BUFFER     "SEND_BUFFER"
#define CMD_CLEAR_BUFFER    "CLEAR_BUFFER"
#define CMD_ENABLE_RELAY    "ENABLE_RELAY"
#define CMD_DISABLE_RELAY   "DISABLE_RELAY"
#define CMD_RETURN_HOME     "RETURN_HOME"
#define CMD_STOP            "STOP"

// Status codes
#define STATUS_OK           0x00
#define STATUS_ERROR        0x01
#define STATUS_BUSY         0x02
#define STATUS_LOW_BATTERY  0x10

// Packet structure
struct LoRaPacket {
  uint8_t type;
  uint16_t counter;
  uint8_t dataLen;
  char data[240];
};

// Protocol constants
#define MAX_PACKET_SIZE     255
#define MAX_DATA_SIZE       240
#define PACKET_TIMEOUT      2000
#define MAX_RETRIES         3

#endif // PROTOCOL_H
