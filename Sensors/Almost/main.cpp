#include <Arduino.h>
#include "SensorManager.h"
#include "GPSHandler.h" 
#include "DataFormatter.h"
#include "sensor_lib.h"

// Global objects using Wiring-like syntax
SensorManager sensors;
GPSHandler gps;
DataFormatter formatter;
SENSOR legacySensors;

// System state
bool systemReady = false;
bool gpsFixed = false;
unsigned long lastDataTime = 0;
const unsigned long DATA_INTERVAL = 100; // 10Hz

// Error handling for legacy sensor lib
uint8_t errorBuffer[5] = {0}; 
bool errorFree = true;
bool ledBuffer[14] = {false};
bool sensorStart = false;

void setup() {
    // Initialize serial for data output only
    Serial.begin(9600);
    
    // Initialize sensor manager
    sensors.begin();
    
    // Initialize legacy sensors 
    sensorStart = true;
    legacySensors.Sensor_begin(&sensorStart, errorBuffer, &errorFree, ledBuffer);
    
    // Initialize GPS
    gps.begin();
    
    // Wait for GPS fix
    while (!gpsFixed) {
        gps.update();
        if (gps.hasValidFix()) {
            gpsFixed = true;
            // Print initial date/time once
            GPSData gpsData = gps.getGPSData();
            printDateTime(gpsData);
        }
        delay(100);
    }
    
    // Validate all sensors
    if (sensors.validateAllSensors()) {
        systemReady = true;
    }
}

void loop() {
    if (!systemReady) return;
    
    unsigned long currentTime = millis();
    
    // Update GPS continuously
    gps.update();
    
    // Main data transmission cycle
    if (currentTime - lastDataTime >= DATA_INTERVAL) {
        
        // Read all sensor data
        SensorReadings readings = sensors.readAllSensors();
        GPSData gpsData = gps.getGPSData();
        
        // Update legacy sensor data
        legacySensors.Motherboard_Temp(ledBuffer, errorBuffer);
        readings.vehicleTemperature = legacySensors.Sensor_Data.Temperature;
        readings.vehicleHumidity = legacySensors.Sensor_Data.Humidity;
        
        // Transmit high priority sensors first
        transmitHighPriority(readings, gpsData);
        
        // Transmit remaining sensors
        transmitStandard(readings, gpsData);
        
        lastDataTime = currentTime;
    }
    
    // APPS safety check
    legacySensors.APPS_Manager();
    
    delay(5);
}

void transmitHighPriority(const SensorReadings& readings, const GPSData& gpsData) {
    uint32_t data;
    
    // ENGINE_TORQUE (ID: 18) - from legacy sensor suspension data as placeholder
    data = formatter.formatQuadSensor(18,
        legacySensors.Sensor_Data.susp_data_1, legacySensors.Sensor_Data.susp_data_2,
        legacySensors.Sensor_Data.susp_data_3, legacySensors.Sensor_Data.susp_data_4);
    Serial.write((uint8_t*)&data, 4);
    
    // BRAKE_POSITION (ID: 8)
    data = formatter.formatSingleSensor(8, readings.brakePosition);
    Serial.write((uint8_t*)&data, 4);
    
    // BRAKE_PRESSURE (ID: 9)
    data = formatter.formatSingleSensor(9, readings.brakePressure);
    Serial.write((uint8_t*)&data, 4);
    
    // ACCELERATOR_POSITION (ID: 10)
    data = formatter.formatSingleSensor(10, readings.acceleratorPosition);
    Serial.write((uint8_t*)&data, 4);
    
    // STEERINGWHEEL_ANGLE (ID: 11)
    data = formatter.formatSingleSensor(11, readings.steeringAngle);
    Serial.write((uint8_t*)&data, 4);
    
    // SPEED (ID: 12)
    data = formatter.formatSingleSensor(12, gpsData.speed);
    Serial.write((uint8_t*)&data, 4);
    
    // SUSPENSION_STROKE (ID: 20)
    data = formatter.formatQuadSensor(20,
        readings.suspensionStroke1, readings.suspensionStroke2,
        readings.suspensionStroke3, readings.suspensionStroke4);
    Serial.write((uint8_t*)&data, 4);
    
    // LATITUDE (ID: 15)
    data = formatter.formatSingleSensor(15, gpsData.latitude);
    Serial.write((uint8_t*)&data, 4);
    
    // LONGITUDE (ID: 16)
    data = formatter.formatSingleSensor(16, gpsData.longitude);
    Serial.write((uint8_t*)&data, 4);
    
    // CAR_CURRENT (ID: 5)
    data = formatter.formatSingleSensor(5, readings.carCurrent);
    Serial.write((uint8_t*)&data, 4);
    
    // BRAKE_CURRENT (ID: 6)
    data = formatter.formatSingleSensor(6, readings.brakeCurrent);
    Serial.write((uint8_t*)&data, 4);
    
    // GYROSCOPE (ID: 29)
    data = formatter.formatTripleSensor(29, readings.gyroX, readings.gyroY, readings.gyroZ);
    Serial.write((uint8_t*)&data, 4);
}

void transmitStandard(const SensorReadings& readings, const GPSData& gpsData) {
    uint32_t data;
    
    // WATER_TEMPERATURE (ID: 2)
    data = formatter.formatDualSensor(2, 
        legacySensors.Sensor_Data.water_temp, readings.waterTemperature2);
    Serial.write((uint8_t*)&data, 4);
    
    // WATER_LOAD/FLOW (ID: 3)
    data = formatter.formatDualSensor(3, readings.waterFlow1, readings.waterFlow2);
    Serial.write((uint8_t*)&data, 4);
    
    // WATER_PRESSURE (ID: 4)
    data = formatter.formatSingleSensor(4, readings.waterPressure);
    Serial.write((uint8_t*)&data, 4);
    
    // VEHICLE_TEMPERATURE_HUMIDITY (ID: 13)
    data = formatter.formatDualSensor(13, readings.vehicleTemperature, readings.vehicleHumidity);
    Serial.write((uint8_t*)&data, 4);
    
    // AIR_TEMPERATURE (ID: 17)
    data = formatter.formatTripleSensor(17,
        legacySensors.Sensor_Data.air_temp_1,
        legacySensors.Sensor_Data.air_temp_2, 
        legacySensors.Sensor_Data.air_temp_3);
    Serial.write((uint8_t*)&data, 4);
    
    // TIRE_TEMPERATURE (ID: 21)
    data = formatter.formatQuadSensor(21,
        legacySensors.Sensor_Data.tire_sens_1, legacySensors.Sensor_Data.tire_sens_2,
        legacySensors.Sensor_Data.tire_sens_3, legacySensors.Sensor_Data.tire_sens_4);
    Serial.write((uint8_t*)&data, 4);
    
    // ACCELEROMETER (ID: 23)
    data = formatter.formatTripleSensor(23, readings.accelX, readings.accelY, readings.accelZ);
    Serial.write((uint8_t*)&data, 4);
    
    // GPS_DATE (ID: 25)
    data = formatter.formatGPSDateTime(25, gpsData);
    Serial.write((uint8_t*)&data, 4);
    
    // GPS_SPEED (ID: 26)
    data = formatter.formatSingleSensor(26, gpsData.speed);
    Serial.write((uint8_t*)&data, 4);
    
    // GPS_ANGLE (ID: 27)
    data = formatter.formatSingleSensor(27, gpsData.course);
    Serial.write((uint8_t*)&data, 4);
    
    // GPS_FIX_AND_QUALITY (ID: 28)
    data = formatter.formatSingleSensor(28, (gpsData.fixQuality << 4) | gpsData.satellites);
    Serial.write((uint8_t*)&data, 4);
    
    // ACTIVATIONS (ID: 22)
    data = formatter.formatActivations(22, readings.fanActive, readings.pumpActive, readings.powerActive);
    Serial.write((uint8_t*)&data, 4);
}

void printDateTime(const GPSData& gps) {
    // Print date and time only once at startup
    if (gps.day < 10) Serial.print("0");
    Serial.print(gps.day);
    if (gps.month < 10) Serial.print("0");
    Serial.print(gps.month);
    if (gps.year < 10) Serial.print("0");
    Serial.print(gps.year % 100);
    Serial.print(" ");
    if (gps.hour < 10) Serial.print("0");
    Serial.print(gps.hour);
    Serial.print(":");
    if (gps.minute < 10) Serial.print("0");
    Serial.print(gps.minute);
    Serial.print(":");
    if (gps.second < 10) Serial.print("0");
    Serial.println(gps.second);
}