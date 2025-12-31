/*
 * BASE STATION FIRMWARE
 * Receives sensor data from main robot via LoRa and displays it
 */

#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include "protocol.h"
#include "app_config.h"

// LoRa pins (same as sub robot)
#define LORA_CS     5
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23

// Status LED
#define STATUS_LED  2

// LoRa communication
uint16_t loraPacketCounter = 0;
int receivedDataCount = 0;

// Forward declarations
bool initLoRa();
void startReceive();
bool receiveLoRaPacket(LoRaPacket* packet);
void handleLoRaPacket(LoRaPacket* packet);
void printSensorData(const char* data);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n====================================");
  Serial.println("   BASE STATION - LoRa Receiver");
  Serial.println("====================================");
  
  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  
  // Initialize LoRa
  if (!initLoRa()) {
    Serial.println("[ERROR] LoRa initialization failed!");
    while (1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(200);
    }
  }
  
  startReceive();
  Serial.println("[LoRa] Ready to receive sensor data");
  Serial.println("====================================\n");
  
  digitalWrite(STATUS_LED, LOW);
}

void loop() {
  // Handle incoming LoRa packets
  LoRaPacket packet;
  if (receiveLoRaPacket(&packet)) {
    handleLoRaPacket(&packet);
    
    // Blink LED on data received
    digitalWrite(STATUS_LED, HIGH);
    delay(50);
    digitalWrite(STATUS_LED, LOW);
  }
  
  delay(10);
}

bool initLoRa() {
  Serial.println("[LoRa] Initializing...");
  Serial.print("[LoRa] Pins: CS=");
  Serial.print(LORA_CS);
  Serial.print(" RST=");
  Serial.print(LORA_RST);
  Serial.print(" DIO0=");
  Serial.println(LORA_DIO0);
  
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

#ifdef LORA_FREQUENCY
  long freq = LORA_FREQUENCY;
#else
  long freq = 433E6;
#endif

  if (!LoRa.begin(freq)) {
    Serial.println("[LoRa] Failed to initialize!");
    return false;
  }

  // Configure LoRa (same settings as robots)
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  
  Serial.print("[LoRa] Frequency: ");
  Serial.print(freq);
  Serial.println(" Hz");
  Serial.println("[LoRa] Initialized successfully!");
  return true;
}

void startReceive() {
  LoRa.receive();
  Serial.println("[LoRa] RX mode active");
}

bool receiveLoRaPacket(LoRaPacket* packet) {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    return false;
  }
  
  // Read packet header
  packet->type = LoRa.read();
  packet->counter = (LoRa.read() << 8) | LoRa.read();
  packet->dataLen = LoRa.read();
  
  // Read data payload
  int idx = 0;
  while (LoRa.available() && idx < packet->dataLen && idx < MAX_DATA_SIZE) {
    packet->data[idx++] = LoRa.read();
  }
  packet->data[idx] = '\0';
  
  // Get signal quality
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  Serial.print("[LoRa RX] Type: 0x");
  Serial.print(packet->type, HEX);
  Serial.print(" | Packet #");
  Serial.print(packet->counter);
  Serial.print(" | RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm | SNR: ");
  Serial.print(snr);
  Serial.println(" dB");
  
  return true;
}

void handleLoRaPacket(LoRaPacket* packet) {
  switch (packet->type) {
    case PKT_DATA:
      Serial.println("\n========== SENSOR DATA RECEIVED ==========");
      printSensorData(packet->data);
      Serial.println("==========================================\n");
      receivedDataCount++;
      Serial.print("[Statistics] Total packets received: ");
      Serial.println(receivedDataCount);
      break;
      
    case PKT_STATUS:
      Serial.print("[Status] ");
      Serial.println(packet->data);
      break;
      
    case PKT_PING:
      Serial.println("[Ping] Received");
      break;
      
    case PKT_PONG:
      Serial.println("[Pong] Received");
      break;
      
    default:
      Serial.print("[Unknown] Type 0x");
      Serial.print(packet->type, HEX);
      Serial.print(": ");
      Serial.println(packet->data);
  }
}

void printSensorData(const char* data) {
  // Parse and display sensor data in readable format
  // Format: TEMP:27.4,HUM:78.7,GAS:0.0,SOIL:45.6,LAT:0.000000,LNG:0.000000,AX:-2.04,AY:-0.44,AZ:9.92,GX:-0.01,GY:0.03,GZ:-0.00
  
  float temp = 0, hum = 0, gas = 0, soil = 0, lat = 0, lng = 0;
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  
  int parsed = sscanf(data, 
    "TEMP:%f,HUM:%f,GAS:%f,SOIL:%f,LAT:%f,LNG:%f,AX:%f,AY:%f,AZ:%f,GX:%f,GY:%f,GZ:%f",
    &temp, &hum, &gas, &soil, &lat, &lng, &ax, &ay, &az, &gx, &gy, &gz);
  
  if (parsed >= 12) {
    // Environmental Data
    Serial.println("📊 Environmental Sensors:");
    Serial.print("  🌡️  Temperature:    ");
    Serial.print(temp, 1);
    Serial.println(" °C");
    Serial.print("  💧 Humidity:       ");
    Serial.print(hum, 1);
    Serial.println(" %");
    Serial.print("  💨 Gas/Air Quality: ");
    Serial.print(gas, 1);
    Serial.println(" %");
    Serial.print("  🌱 Soil Moisture:  ");
    Serial.print(soil, 1);
    Serial.println(" %");
    
    // GPS Data
    Serial.println("\n📍 GPS Location:");
    Serial.print("  Latitude:  ");
    Serial.println(lat, 6);
    Serial.print("  Longitude: ");
    Serial.println(lng, 6);
    
    // IMU Data
    Serial.println("\n🎯 IMU Sensor:");
    Serial.print("  Acceleration: X=");
    Serial.print(ax, 2);
    Serial.print(" Y=");
    Serial.print(ay, 2);
    Serial.print(" Z=");
    Serial.print(az, 2);
    Serial.println(" m/s²");
    Serial.print("  Gyroscope:    X=");
    Serial.print(gx, 3);
    Serial.print(" Y=");
    Serial.print(gy, 3);
    Serial.print(" Z=");
    Serial.print(gz, 3);
    Serial.println(" rad/s");
  } else {
    // If parsing failed, just print raw data
    Serial.print("Raw Data: ");
    Serial.println(data);
  }
}
