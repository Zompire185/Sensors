#include "SensorManager.h"

// Global pointer for ISR access
SensorManager* globalSensorPtr = nullptr;

// ISR functions
void flowSensor1ISR() {
    if (globalSensorPtr) {
        globalSensorPtr->readings.flowPulseCount1++;
    }
}

void flowSensor2ISR() {
    if (globalSensorPtr) {
        globalSensorPtr->readings.flowPulseCount2++;
    }
}

// Constructor
SensorManager::SensorManager() {
    globalSensorPtr = this;
    dht = new DHT(DHT_PIN, DHT_TYPE);
    mpu = new MPU6050();
    memset(&readings, 0, sizeof(readings));
}

// Destructor
SensorManager::~SensorManager() {
    delete dht;
    delete mpu;
}

bool SensorManager::begin() {
    // Initialize I2C
    Wire.begin();
    
    // Initialize DHT sensor
    dht->begin();
    
    // Initialize MPU6050
    mpu->initialize();
    if (!mpu->testConnection()) {
        return false;
    }
    
    // Set up analog pins
    pinMode(WATER_TEMP_2_PIN, INPUT);
    pinMode(INVERTER_TEMP_PIN, INPUT);
    pinMode(WATER_PRESSURE_PIN, INPUT);
    pinMode(BRAKE_PRESSURE_PIN, INPUT);
    pinMode(CAR_CURRENT_PIN, INPUT);
    pinMode(BRAKE_CURRENT_PIN, INPUT);
    pinMode(SUSPENSION_1_PIN, INPUT);
    pinMode(SUSPENSION_2_PIN, INPUT);
    pinMode(SUSPENSION_3_PIN, INPUT);
    pinMode(SUSPENSION_4_PIN, INPUT);
    pinMode(BRAKE_POSITION_PIN, INPUT);
    pinMode(ACCELERATOR_POSITION_PIN, INPUT);
    pinMode(STEERING_ANGLE_PIN, INPUT);
    
    // Set up digital pins for flow sensors
    pinMode(WATER_FLOW_1_PIN, INPUT_PULLUP);
    pinMode(WATER_FLOW_2_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(WATER_FLOW_1_PIN), flowSensor1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(WATER_FLOW_2_PIN), flowSensor2ISR, RISING);
    
    // Set up activation control pins
    pinMode(FAN_CONTROL_PIN, OUTPUT);
    pinMode(PUMP_CONTROL_PIN, OUTPUT);
    pinMode(POWER_CONTROL_PIN, OUTPUT);
    
    digitalWrite(FAN_CONTROL_PIN, LOW);
    digitalWrite(PUMP_CONTROL_PIN, LOW);
    digitalWrite(POWER_CONTROL_PIN, LOW);
    
    return true;
}

float SensorManager::readNTCTemperature(int pin, float beta) {
    int adcValue = analogRead(pin);
    float voltage = (adcValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
    
    float resistance = (NTC_R25 * voltage) / (REFERENCE_VOLTAGE - voltage);
    float temperature = (beta * NTC_T25) / (beta + NTC_T25 * log(resistance / NTC_R25)) - 273.15;
    
    return temperature;
}

float SensorManager::readPressure(int pin, float maxPressure) {
    int adcValue = analogRead(pin);
    float voltage = (adcValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
    
    float pressureRatio = (voltage - PRESSURE_MIN_VOLTAGE) / (PRESSURE_MAX_VOLTAGE - PRESSURE_MIN_VOLTAGE);
    return pressureRatio * maxPressure;
}

float SensorManager::readCurrent(int pin) {
    int adcValue = analogRead(pin);
    float voltage = (adcValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
    
    float current = (voltage - (REFERENCE_VOLTAGE / 2.0)) / CURRENT_SENSITIVITY;
    return current;
}

float SensorManager::readLinearPosition(int pin) {
    int adcValue = analogRead(pin);
    float voltage = (adcValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
    
    float positionRatio = voltage / REFERENCE_VOLTAGE;
    return positionRatio * SUSPENSION_STROKE_MM;
}

float SensorManager::readWaterTemperature2() {
    return readNTCTemperature(WATER_TEMP_2_PIN);
}

float SensorManager::readInverterTemperature() {
    return readNTCTemperature(INVERTER_TEMP_PIN);
}

float SensorManager::readWaterPressure() {
    return readPressure(WATER_PRESSURE_PIN, WATER_PRESSURE_MAX_PSI);
}

float SensorManager::readBrakePressure() {
    return readPressure(BRAKE_PRESSURE_PIN, BRAKE_PRESSURE_MAX_PSI);
}

float SensorManager::readCarCurrent() {
    return readCurrent(CAR_CURRENT_PIN);
}

float SensorManager::readBrakeCurrent() {
    return readCurrent(BRAKE_CURRENT_PIN);
}

float SensorManager::readBrakePosition() {
    return readLinearPosition(BRAKE_POSITION_PIN);
}

float SensorManager::readAcceleratorPosition() {
    return readLinearPosition(ACCELERATOR_POSITION_PIN);
}

float SensorManager::readSteeringAngle() {
    int adcValue = analogRead(STEERING_ANGLE_PIN);
    float voltage = (adcValue * REFERENCE_VOLTAGE) / ADC_RESOLUTION;
    
    float angle = ((voltage / REFERENCE_VOLTAGE) - 0.5) * 360.0;
    return angle;
}

float SensorManager::readSuspensionStroke(int sensor) {
    int pin;
    switch(sensor) {
        case 1: pin = SUSPENSION_1_PIN; break;
        case 2: pin = SUSPENSION_2_PIN; break;
        case 3: pin = SUSPENSION_3_PIN; break;
        case 4: pin = SUSPENSION_4_PIN; break;
        default: return 0.0;
    }
    return readLinearPosition(pin);
}

void SensorManager::readIMU() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    readings.accelX = ax / 16384.0;
    readings.accelY = ay / 16384.0;
    readings.accelZ = az / 16384.0;
    
    readings.gyroX = gx / 131.0;
    readings.gyroY = gy / 131.0;
    readings.gyroZ = gz / 131.0;
}

void SensorManager::readVehicleEnvironment() {
    readings.vehicleTemperature = dht->readTemperature();
    readings.vehicleHumidity = dht->readHumidity();
    
    if (isnan(readings.vehicleTemperature)) readings.vehicleTemperature = 0.0;
    if (isnan(readings.vehicleHumidity)) readings.vehicleHumidity = 0.0;
}

void SensorManager::updateFlowSensors() {
    unsigned long currentTime = millis();
    
    if (currentTime - readings.lastFlowUpdate >= 1000) {
        float deltaTime = (currentTime - readings.lastFlowUpdate) / 1000.0;
        
        readings.waterFlow1 = (readings.flowPulseCount1 / (FLOW_CALIBRATION_FACTOR * deltaTime / 60.0));
        readings.waterFlow2 = (readings.flowPulseCount2 / (FLOW_CALIBRATION_FACTOR * deltaTime / 60.0));
        
        readings.flowPulseCount1 = 0;
        readings.flowPulseCount2 = 0;
        readings.lastFlowUpdate = currentTime;
    }
}

void SensorManager::updateActivations() {
    readings.fanActive = digitalRead(FAN_CONTROL_PIN);
    readings.pumpActive = digitalRead(PUMP_CONTROL_PIN);
    readings.powerActive = digitalRead(POWER_CONTROL_PIN);
}

SensorReadings SensorManager::readAllSensors() {
    // Read analog sensors
    readings.waterTemperature2 = readWaterTemperature2();
    readings.inverterTemperature = readInverterTemperature();
    readings.waterPressure = readWaterPressure();
    readings.brakePressure = readBrakePressure();
    readings.carCurrent = readCarCurrent();
    readings.brakeCurrent = readBrakeCurrent();
    readings.brakePosition = readBrakePosition();
    readings.acceleratorPosition = readAcceleratorPosition();
    readings.steeringAngle = readSteeringAngle();
    
    // Read suspension sensors
    readings.suspensionStroke1 = readSuspensionStroke(1);
    readings.suspensionStroke2 = readSuspensionStroke(2);
    readings.suspensionStroke3 = readSuspensionStroke(3);
    readings.suspensionStroke4 = readSuspensionStroke(4);
    
    // Read IMU
    readIMU();
    
    // Read environmental sensors
    readVehicleEnvironment();
    
    // Update flow sensors
    updateFlowSensors();
    
    // Update activation states
    updateActivations();
    
    return readings;
}

bool SensorManager::validateAllSensors() {
    SensorReadings testReadings = readAllSensors();
    
    // Validate temperature readings
    if (testReadings.waterTemperature2 < -50 || testReadings.waterTemperature2 > 150) {
        return false;
    }
    
    if (testReadings.inverterTemperature < -50 || testReadings.inverterTemperature > 150) {
        return false;
    }
    
    // Validate pressure readings
    if (testReadings.waterPressure < 0 || testReadings.waterPressure > WATER_PRESSURE_MAX_PSI) {
        return false;
    }
    
    if (testReadings.brakePressure < 0 || testReadings.brakePressure > BRAKE_PRESSURE_MAX_PSI) {
        return false;
    }
    
    // Validate current readings
    if (abs(testReadings.carCurrent) > CURRENT_MAX_AMPS) {
        return false;
    }
    
    // Validate position readings
    if (testReadings.brakePosition < 0 || testReadings.brakePosition > 100) {
        return false;
    }
    
    if (testReadings.acceleratorPosition < 0 || testReadings.acceleratorPosition > 100) {
        return false;
    }
    
    // Validate steering angle
    if (testReadings.steeringAngle < -180 || testReadings.steeringAngle > 180) {
        return false;
    }
    
    // Validate suspension stroke readings
    if (testReadings.suspensionStroke1 < 0 || testReadings.suspensionStroke1 > SUSPENSION_STROKE_MM) {
        return false;
    }
    
    if (testReadings.suspensionStroke2 < 0 || testReadings.suspensionStroke2 > SUSPENSION_STROKE_MM) {
        return false;
    }
    
    if (testReadings.suspensionStroke3 < 0 || testReadings.suspensionStroke3 > SUSPENSION_STROKE_MM) {
        return false;
    }
    
    if (testReadings.suspensionStroke4 < 0 || testReadings.suspensionStroke4 > SUSPENSION_STROKE_MM) {
        return false;
    }
    
    // Check for NaN IMU readings
    if (isnan(testReadings.accelX) || isnan(testReadings.accelY) || isnan(testReadings.accelZ)) {
        return false;
    }
    
    if (isnan(testReadings.gyroX) || isnan(testReadings.gyroY) || isnan(testReadings.gyroZ)) {
        return false;
    }
    
    return true;
}