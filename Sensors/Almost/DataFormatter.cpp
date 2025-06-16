#include "DataFormatter.h"

// Constructor
DataFormatter::DataFormatter() {
    // Nothing to initialize
}

uint32_t DataFormatter::scaleTemperature(float temp, uint8_t bits) {
    return SCALE_SIGNED_VALUE(temp, bits, TEMPERATURE_MIN, TEMPERATURE_MAX);
}

uint32_t DataFormatter::scalePressure(float pressure, uint8_t bits) {
    return SCALE_SIGNED_VALUE(pressure, bits, PRESSURE_MIN, PRESSURE_MAX);
}

uint32_t DataFormatter::scaleCurrent(float current, uint8_t bits) {
    return SCALE_SIGNED_VALUE(current, bits, CURRENT_MIN, CURRENT_MAX);
}

uint32_t DataFormatter::scalePosition(float position, uint8_t bits) {
    return SCALE_SIGNED_VALUE(position, bits, POSITION_MIN, POSITION_MAX);
}

uint32_t DataFormatter::scaleAngle(float angle, uint8_t bits) {
    return SCALE_SIGNED_VALUE(angle, bits, ANGLE_MIN, ANGLE_MAX);
}

uint32_t DataFormatter::scaleSpeed(float speed, uint8_t bits) {
    return SCALE_SIGNED_VALUE(speed, bits, SPEED_MIN, SPEED_MAX);
}

uint32_t DataFormatter::scaleLatitude(float latitude, uint8_t bits) {
    return SCALE_SIGNED_VALUE(latitude, bits, LATITUDE_MIN, LATITUDE_MAX);
}

uint32_t DataFormatter::scaleLongitude(float longitude, uint8_t bits) {
    return SCALE_SIGNED_VALUE(longitude, bits, LONGITUDE_MIN, LONGITUDE_MAX);
}

uint32_t DataFormatter::scaleAcceleration(float accel, uint8_t bits) {
    return SCALE_SIGNED_VALUE(accel, bits, ACCELERATION_MIN, ACCELERATION_MAX);
}

uint32_t DataFormatter::scaleGyroscope(float gyro, uint8_t bits) {
    return SCALE_SIGNED_VALUE(gyro, bits, GYRO_MIN, GYRO_MAX);
}

uint32_t DataFormatter::scaleFlow(float flow, uint8_t bits) {
    return SCALE_SIGNED_VALUE(flow, bits, FLOW_MIN, FLOW_MAX);
}

uint32_t DataFormatter::scaleHumidity(float humidity, uint8_t bits) {
    return SCALE_SIGNED_VALUE(humidity, bits, HUMIDITY_MIN, HUMIDITY_MAX);
}

uint32_t DataFormatter::formatSingleSensor(uint8_t sensorId, float value) {
    uint32_t result = 0;
    
    // Set sensor ID in bits 31-27
    SET_SENSOR_ID(result, sensorId);
    
    // Scale value to 27 bits based on sensor type
    uint32_t scaledValue = 0;
    
    switch(sensorId) {
        case 4: // WATER_PRESSURE
        case 9: // BRAKE_PRESSURE
            scaledValue = scalePressure(value, 27);
            break;
        case 5: // CAR_CURRENT
        case 6: // BRAKE_CURRENT
            scaledValue = scaleCurrent(value, 27);
            break;
        case 7: // INVERTER_TEMPERATURE
            scaledValue = scaleTemperature(value, 27);
            break;
        case 8: // BRAKE_POSITION
        case 10: // ACCELERATOR_POSITION
            scaledValue = scalePosition(value, 27);
            break;
        case 11: // STEERINGWHEEL_ANGLE
        case 27: // GPS_ANGLE
            scaledValue = scaleAngle(value, 27);
            break;
        case 12: // SPEED
        case 26: // GPS_SPEED
            scaledValue = scaleSpeed(value, 27);
            break;
        case 15: // LATITUDE
            scaledValue = scaleLatitude(value, 27);
            break;
        case 16: // LONGITUDE
            scaledValue = scaleLongitude(value, 27);
            break;
        case 24: // INVERTER_VOLTAGE
            scaledValue = SCALE_VALUE(value / 1000.0, 27);
            break;
        case 28: // GPS_FIX_AND_QUALITY
            scaledValue = (uint32_t)value & 0x7FFFFFF;
            break;
        default:
            scaledValue = SCALE_VALUE(value, 27);
            break;
    }
    
    // Set the scaled value in bits 26-0
    result |= (scaledValue & 0x7FFFFFF);
    
    return result;
}

uint32_t DataFormatter::formatDualSensor(uint8_t sensorId, float value1, float value2) {
    uint32_t result = 0;
    
    // Set sensor ID in bits 31-27
    SET_SENSOR_ID(result, sensorId);
    
    // Scale values based on sensor type
    uint32_t scaledValue1 = 0, scaledValue2 = 0;
    
    switch(sensorId) {
        case 2: // WATER_TEMPERATURE
            scaledValue1 = scaleTemperature(value1, 13);
            scaledValue2 = scaleTemperature(value2, 13);
            break;
        case 3: // WATER_LOAD/FLOW
            scaledValue1 = scaleFlow(value1, 13);
            scaledValue2 = scaleFlow(value2, 13);
            break;
        case 13: // VEHICLE_TEMPERATURE_HUMIDITY
            scaledValue1 = scaleTemperature(value1, 13);
            scaledValue2 = scaleHumidity(value2, 13);
            break;
        default:
            scaledValue1 = SCALE_VALUE(value1, 13);
            scaledValue2 = SCALE_VALUE(value2, 13);
            break;
    }
    
    // Set sensor 1 data in bits 26-14
    result |= ((scaledValue1 & 0x1FFF) << 14);
    
    // Bit 13 is separator (0)
    SET_SEPARATOR_BIT(result, 13);
    
    // Set sensor 2 data in bits 12-0
    result |= (scaledValue2 & 0x1FFF);
    
    return result;
}

uint32_t DataFormatter::formatTripleSensor(uint8_t sensorId, float value1, float value2, float value3) {
    uint32_t result = 0;
    
    // Set sensor ID in bits 31-27
    SET_SENSOR_ID(result, sensorId);
    
    // Scale values based on sensor type
    uint32_t scaledValue1 = 0, scaledValue2 = 0, scaledValue3 = 0;
    
    switch(sensorId) {
        case 17: // AIR_TEMPERATURE
            scaledValue1 = scaleTemperature(value1, 9);
            scaledValue2 = scaleTemperature(value2, 9);
            scaledValue3 = scaleTemperature(value3, 9);
            break;
        case 23: // ACCELEROMETER
            scaledValue1 = scaleAcceleration(value1, 9);
            scaledValue2 = scaleAcceleration(value2, 9);
            scaledValue3 = scaleAcceleration(value3, 9);
            break;
        case 29: // GYROSCOPE
            scaledValue1 = scaleGyroscope(value1, 9);
            scaledValue2 = scaleGyroscope(value2, 9);
            scaledValue3 = scaleGyroscope(value3, 9);
            break;
        default:
            scaledValue1 = SCALE_VALUE(value1, 9);
            scaledValue2 = SCALE_VALUE(value2, 9);
            scaledValue3 = SCALE_VALUE(value3, 9);
            break;
    }
    
    // Set values in bits 26-18, 17-9, 8-0
    result |= ((scaledValue1 & 0x1FF) << 18);
    result |= ((scaledValue2 & 0x1FF) << 9);
    result |= (scaledValue3 & 0x1FF);
    
    return result;
}

uint32_t DataFormatter::formatQuadSensor(uint8_t sensorId, float value1, float value2, float value3, float value4) {
    uint32_t result = 0;
    
    // Set sensor ID in bits 31-27
    SET_SENSOR_ID(result, sensorId);
    
    // Scale values based on sensor type (6 bits each with separators)
    uint32_t scaledValue1 = 0, scaledValue2 = 0, scaledValue3 = 0, scaledValue4 = 0;
    
    switch(sensorId) {
        case 18: // ENGINE_TORQUE
            scaledValue1 = SCALE_VALUE(value1, 6);
            scaledValue2 = SCALE_VALUE(value2, 6);
            scaledValue3 = SCALE_VALUE(value3, 6);
            scaledValue4 = SCALE_VALUE(value4, 6);
            break;
        case 19: // ENGINE_TEMPERATURE
        case 21: // TIRE_TEMPERATURE
            scaledValue1 = scaleTemperature(value1, 6);
            scaledValue2 = scaleTemperature(value2, 6);
            scaledValue3 = scaleTemperature(value3, 6);
            scaledValue4 = scaleTemperature(value4, 6);
            break;
        case 20: // SUSPENSION_STROKE
            scaledValue1 = scalePosition(value1, 6);
            scaledValue2 = scalePosition(value2, 6);
            scaledValue3 = scalePosition(value3, 6);
            scaledValue4 = scalePosition(value4, 6);
            break;
        default:
            scaledValue1 = SCALE_VALUE(value1, 6);
            scaledValue2 = SCALE_VALUE(value2, 6);
            scaledValue3 = SCALE_VALUE(value3, 6);
            scaledValue4 = SCALE_VALUE(value4, 6);
            break;
    }
    
    // Format: Bit 31-27 Sensor ID, Bit 26-21 Sensor 1, Bit 20 Separator (0), 
    // Bit 19-14 Sensor 2, Bit 13 Separator (0), Bit 12-7 Sensor 3, 
    // Bit 6 Separator (0), Bit 5-0 Sensor 4
    result |= ((scaledValue1 & 0x3F) << 21);
    SET_SEPARATOR_BIT(result, 20);
    result |= ((scaledValue2 & 0x3F) << 14);
    SET_SEPARATOR_BIT(result, 13);
    result |= ((scaledValue3 & 0x3F) << 7);
    SET_SEPARATOR_BIT(result, 6);
    result |= (scaledValue4 & 0x3F);
    
    return result;
}

uint32_t DataFormatter::formatTemperatureHumidity(uint8_t sensorId, float temperature, float humidity) {
    return formatDualSensor(sensorId, temperature, humidity);
}

uint32_t DataFormatter::formatAccelerometer(float accelX, float accelY, float accelZ) {
    return formatTripleSensor(23, accelX, accelY, accelZ);
}

uint32_t DataFormatter::formatGyroscope(float gyroX, float gyroY, float gyroZ) {
    return formatTripleSensor(29, gyroX, gyroY, gyroZ);
}

uint32_t DataFormatter::formatGPSDateTime(uint8_t sensorId, const GPSData& gpsData) {
    uint32_t result = 0;
    
    // Set sensor ID in bits 31-27
    SET_SENSOR_ID(result, sensorId);
    
    // Pack date into 13 bits: YYMMDD
    uint32_t dateValue = ((gpsData.year - 2000) << 8) | (gpsData.month << 4) | gpsData.day;
    
    // Pack time into 13 bits: HHMMSS (compressed)
    uint32_t timeValue = (gpsData.hour << 8) | (gpsData.minute << 2) | (gpsData.second / 15);
    
    // Set date in bits 26-14
    result |= ((dateValue & 0x1FFF) << 14);
    
    // Bit 13 is separator (0)
    SET_SEPARATOR_BIT(result, 13);
    
    // Set time in bits 12-0
    result |= (timeValue & 0x1FFF);
    
    return result;
}

uint32_t DataFormatter::formatActivations(uint8_t sensorId, bool fan, bool pump, bool power) {
    uint32_t result = 0;
    
    // Set sensor ID in bits 31-27
    SET_SENSOR_ID(result, sensorId);
    
    // Pack boolean flags into lower bits
    uint32_t activationValue = 0;
    if (fan) activationValue |= 0x01;
    if (pump) activationValue |= 0x02;
    if (power) activationValue |= 0x04;
    
    // Set activation data in bits 26-0
    result |= (activationValue & 0x7FFFFFF);
    
    return result;
}