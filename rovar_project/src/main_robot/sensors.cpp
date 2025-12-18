/*
 * Sensor Manager Implementation
 */

#include "sensors.h"
#include "main_config.h"
#include <TinyGPS++.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <HardwareSerial.h>

// Sensor objects
DHT dht(AIR_DHT_PIN, DHT22);
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial co2Serial(2);

// Current sensor readings
SensorData currentData;

void initSensors() {
  Serial.println("Initializing sensors...");
  
  // Analog sensors
  pinMode(WATER_TEMP_PIN, INPUT);
  pinMode(WATER_PH_PIN, INPUT);
  pinMode(WATER_TURBIDITY_PIN, INPUT);
  pinMode(WATER_DO_PIN, INPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  pinMode(SOIL_TEMP_PIN, INPUT);
  pinMode(SOIL_PH_PIN, INPUT);
  pinMode(BATTERY_PIN, INPUT);
  
  // DHT sensor
  dht.begin();
  
  // BMP280 sensor
  Wire.begin(AIR_BMP_SDA, AIR_BMP_SCL);
  if (!bmp.begin(0x76)) {
    Serial.println("Warning: BMP280 not found");
  }
  
  // GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // CO2 sensor
  co2Serial.begin(9600, SERIAL_8N1, AIR_CO2_RX, AIR_CO2_TX);
  
  Serial.println("Sensors initialized");
}

void updateSensors() {
  currentData.timestamp = millis();
  
  // Water sensors (analog readings - simplified)
  currentData.waterTemp = analogRead(WATER_TEMP_PIN) * 0.1; // Adjust based on your sensor
  currentData.waterPH = (analogRead(WATER_PH_PIN) / 4095.0) * 14.0;
  currentData.waterTurbidity = analogRead(WATER_TURBIDITY_PIN) * 0.5;
  currentData.waterDO = (analogRead(WATER_DO_PIN) / 4095.0) * 20.0;
  
  // Soil sensors
  currentData.soilMoisture = map(analogRead(SOIL_MOISTURE_PIN), 0, 4095, 100, 0);
  currentData.soilTemp = analogRead(SOIL_TEMP_PIN) * 0.1;
  currentData.soilPH = (analogRead(SOIL_PH_PIN) / 4095.0) * 14.0;
  currentData.soilN = 0.0;  // TODO: Implement NPK sensor reading
  currentData.soilP = 0.0;
  currentData.soilK = 0.0;
  
  // Air sensors
  currentData.airTemp = dht.readTemperature();
  currentData.airHumidity = dht.readHumidity();
  currentData.airPressure = bmp.readPressure() / 100.0;
  currentData.airCO2 = 400.0; // TODO: Implement CO2 sensor reading
  
  // GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  if (gps.location.isValid()) {
    currentData.latitude = gps.location.lat();
    currentData.longitude = gps.location.lng();
    currentData.altitude = gps.altitude.meters();
    currentData.gpsValid = true;
  } else {
    currentData.gpsValid = false;
  }
  
  // Battery
  int rawBattery = analogRead(BATTERY_PIN);
  currentData.batteryVoltage = (rawBattery / 4095.0) * 3.3 * 3.0; // Voltage divider
  currentData.batteryPercent = ((currentData.batteryVoltage - BATTERY_MIN_VOLTAGE) / 
                               (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  currentData.batteryPercent = constrain(currentData.batteryPercent, 0.0, 100.0);
}

SensorData getSensorData() {
  return currentData;
}

String getSensorDataJSON() {
  String json = "{";
  json += "\"timestamp\":" + String(currentData.timestamp) + ",";
  json += "\"water\":{";
  json += "\"temp\":" + String(currentData.waterTemp, 2) + ",";
  json += "\"ph\":" + String(currentData.waterPH, 2) + ",";
  json += "\"turbidity\":" + String(currentData.waterTurbidity, 2) + ",";
  json += "\"do\":" + String(currentData.waterDO, 2);
  json += "},";
  json += "\"soil\":{";
  json += "\"moisture\":" + String(currentData.soilMoisture, 1) + ",";
  json += "\"temp\":" + String(currentData.soilTemp, 2) + ",";
  json += "\"ph\":" + String(currentData.soilPH, 2);
  json += "},";
  json += "\"air\":{";
  json += "\"temp\":" + String(currentData.airTemp, 2) + ",";
  json += "\"humidity\":" + String(currentData.airHumidity, 1) + ",";
  json += "\"pressure\":" + String(currentData.airPressure, 2) + ",";
  json += "\"co2\":" + String(currentData.airCO2, 1);
  json += "},";
  json += "\"gps\":{";
  json += "\"lat\":" + String(currentData.latitude, 6) + ",";
  json += "\"lon\":" + String(currentData.longitude, 6) + ",";
  json += "\"valid\":" + String(currentData.gpsValid ? "true" : "false");
  json += "},";
  json += "\"battery\":{";
  json += "\"voltage\":" + String(currentData.batteryVoltage, 2) + ",";
  json += "\"percent\":" + String(currentData.batteryPercent, 1);
  json += "}";
  json += "}";
  return json;
}
