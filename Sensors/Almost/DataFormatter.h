#ifndef DATAFORMATTER_H
#define DATAFORMATTER_H

#include <Arduino.h>
#include "GPSHandler.h"

// Bit manipulation macros
#define SET_SENSOR_ID(data, id) ((data) |= ((uint32_t)(id) << 27))
#define SET_SEPARATOR_BIT(data, pos) ((data) &= ~((uint32_t)1 << (pos)))
#define SCALE_VALUE(value, bits) ((uint32_t)((value) * ((1UL << (bits)) - 1)))
#define SCALE_SIGNED_VALUE(value, bits, min_val, max_val) \
    ((uint32_t)(((value) - (min_val)) / ((max_val) - (min_val)) * ((1UL << (bits)) - 1)))

class DataFormatter {
private:
    // Scaling constants
    static constexpr float TEMPERATURE_MIN = -50.0;
    static constexpr float TEMPERATURE_MAX = 200.0;
    static constexpr float PRESSURE_MIN = 0.0;
    static constexpr float PRESSURE_MAX = 1000.0;
    static constexpr float CURRENT_MIN = -200.0;
    static constexpr float CURRENT_MAX = 200.0;
    static constexpr float POSITION_MIN = 0.0;
    static constexpr float POSITION_MAX = 100.0;
    static constexpr float ANGLE_MIN = -180.0;
    static constexpr float ANGLE_MAX = 180.0;
    static constexpr float SPEED_MIN = 0.0;
    static constexpr float SPEED_MAX = 300.0;
    static constexpr float LATITUDE_MIN = -90.0;
    static constexpr float LATITUDE_MAX = 90.0;
    static constexpr float LONGITUDE_MIN = -180.0;
    static constexpr float LONGITUDE_MAX = 180.0;
    static constexpr float ACCELERATION_MIN = -20.0;
    static constexpr float ACCELERATION_MAX = 20.0;
    static constexpr float GYRO_MIN = -2000.0;
    static constexpr float GYRO_MAX = 2000.0;
    static constexpr float FLOW_MIN = 0.0;
    static constexpr float FLOW_MAX = 30.0;
    static constexpr float HUMIDITY_MIN = 0.0;
    static constexpr float HUMIDITY_MAX = 100.0;
    
    // Helper methods - DECLARED HERE
    uint32_t scaleTemperature(float temp, uint8_t bits);
    uint32_t scalePressure(float pressure, uint8_t bits);
    uint32_t scaleCurrent(float current, uint8_t bits);
    uint32_t scalePosition(float position, uint8_t bits);
    uint32_t scaleAngle(float angle, uint8_t bits);
    uint32_t scaleSpeed(float speed, uint8_t bits);
    uint32_t scaleLatitude(float latitude, uint8_t bits);
    uint32_t scaleLongitude(float longitude, uint8_t bits);
    uint32_t scaleAcceleration(float accel, uint8_t bits);
    uint32_t scaleGyroscope(float gyro, uint8_t bits);
    uint32_t scaleFlow(float flow, uint8_t bits);
    uint32_t scaleHumidity(float humidity, uint8_t bits);
    
public:
    // Constructor
    DataFormatter();
    
    // Public methods - DECLARED HERE
    uint32_t formatSingleSensor(uint8_t sensorId, float value);
    uint32_t formatDualSensor(uint8_t sensorId, float value1, float value2);
    uint32_t formatTripleSensor(uint8_t sensorId, float value1, float value2, float value3);
    uint32_t formatQuadSensor(uint8_t sensorId, float value1, float value2, float value3, float value4);
    uint32_t formatTemperatureHumidity(uint8_t sensorId, float temperature, float humidity);
    uint32_t formatAccelerometer(float accelX, float accelY, float accelZ);
    uint32_t formatGyroscope(float gyroX, float gyroY, float gyroZ);
    uint32_t formatGPSDateTime(uint8_t sensorId, const GPSData& gpsData);
    uint32_t formatActivations(uint8_t sensorId, bool fan, bool pump, bool power);
};

#endif