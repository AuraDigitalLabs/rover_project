/*
 * Sensor Manager for Main Robot
 * Handles all sensor readings
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// Sensor data structure
struct SensorData {
  // Water quality
  float waterTemp;
  float waterPH;
  float waterTurbidity;
  float waterDO;
  
  // Soil
  float soilMoisture;
  float soilTemp;
  float soilPH;
  float soilN;
  float soilP;
  float soilK;
  
  // Air quality
  float airTemp;
  float airHumidity;
  float airPressure;
  float airCO2;
  
  // GPS
  float latitude;
  float longitude;
  float altitude;
  bool gpsValid;
  
  // System
  float batteryVoltage;
  float batteryPercent;
  unsigned long timestamp;
};

// Function declarations
void initSensors();
void updateSensors();
SensorData getSensorData();
String getSensorDataJSON();

#endif // SENSORS_H
