#include "GPSHandler.h"
#include <string.h>
#include <stdlib.h>

// Constructor
GPSHandler::GPSHandler() {
    gpsSerial = new SoftwareSerial(GPS_RX_PIN, GPS_TX_PIN);
    memset(&gpsData, 0, sizeof(gpsData));
    bufferIndex = 0;
    memset(nmeaBuffer, 0, GPS_BUFFER_SIZE);
}

// Destructor
GPSHandler::~GPSHandler() {
    delete gpsSerial;
}

bool GPSHandler::begin() {
    // Initialize GPS control pins
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    pinMode(GPS_3D_FIX_PIN, INPUT);
    
    // Enable GPS module
    enableGPS();
    delay(100);
    
    // Initialize serial communication
    gpsSerial->begin(GPS_BAUD_RATE);
    
    // Wait for GPS to start responding
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        if (gpsSerial->available()) {
            return true;
        }
        delay(100);
    }
    
    return false;
}

void GPSHandler::enableGPS() {
    digitalWrite(GPS_ENABLE_PIN, HIGH);
}

void GPSHandler::disableGPS() {
    digitalWrite(GPS_ENABLE_PIN, LOW);
}

bool GPSHandler::is3DFixAvailable() {
    return digitalRead(GPS_3D_FIX_PIN) == HIGH;
}

void GPSHandler::update() {
    while (gpsSerial->available()) {
        char c = gpsSerial->read();
        
        if (c == '$') {
            bufferIndex = 0;
            nmeaBuffer[bufferIndex++] = c;
        } else if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                nmeaBuffer[bufferIndex] = '\0';
                
                if (validateChecksum(nmeaBuffer)) {
                    if (strncmp(nmeaBuffer, "$GPGGA", 6) == 0 || strncmp(nmeaBuffer, "$GNGGA", 6) == 0) {
                        parseGGA(nmeaBuffer);
                    } else if (strncmp(nmeaBuffer, "$GPRMC", 6) == 0 || strncmp(nmeaBuffer, "$GNRMC", 6) == 0) {
                        parseRMC(nmeaBuffer);
                    } else if (strncmp(nmeaBuffer, "$GPGSA", 6) == 0 || strncmp(nmeaBuffer, "$GNGSA", 6) == 0) {
                        parseGSA(nmeaBuffer);
                    } else if (strncmp(nmeaBuffer, "$GPVTG", 6) == 0 || strncmp(nmeaBuffer, "$GNVTG", 6) == 0) {
                        parseVTG(nmeaBuffer);
                    }
                }
            }
            bufferIndex = 0;
        } else if (bufferIndex < GPS_BUFFER_SIZE - 1) {
            nmeaBuffer[bufferIndex++] = c;
        }
    }
}

bool GPSHandler::validateChecksum(const char* sentence) {
    if (strlen(sentence) < 4) return false;
    
    const char* checksumPtr = strrchr(sentence, '*');
    if (!checksumPtr) return false;
    
    uint8_t calculatedChecksum = 0;
    for (const char* ptr = sentence + 1; ptr < checksumPtr; ptr++) {
        calculatedChecksum ^= *ptr;
    }
    
    uint8_t providedChecksum = strtol(checksumPtr + 1, NULL, 16);
    
    return calculatedChecksum == providedChecksum;
}

bool GPSHandler::parseGGA(const char* sentence) {
    char buffer[MAX_NMEA_LENGTH];
    strncpy(buffer, sentence, sizeof(buffer));
    
    char* token = strtok(buffer, ",");
    int fieldIndex = 0;
    
    while (token != NULL) {
        switch (fieldIndex) {
            case 1: {
                if (strlen(token) >= 6) {
                    gpsData.hour = (token[0] - '0') * 10 + (token[1] - '0');
                    gpsData.minute = (token[2] - '0') * 10 + (token[3] - '0');
                    gpsData.second = (token[4] - '0') * 10 + (token[5] - '0');
                    if (strlen(token) > 7) {
                        gpsData.millisecond = ((token[7] - '0') * 100 + (token[8] - '0') * 10);
                    }
                }
                break;
            }
            case 2:
                if (strlen(token) > 0) {
                    gpsData.latitude = parseCoordinate(token, 'N');
                }
                break;
            case 3:
                if (token[0] == 'S') {
                    gpsData.latitude = -gpsData.latitude;
                }
                break;
            case 4:
                if (strlen(token) > 0) {
                    gpsData.longitude = parseCoordinate(token, 'E');
                }
                break;
            case 5:
                if (token[0] == 'W') {
                    gpsData.longitude = -gpsData.longitude;
                }
                break;
            case 6:
                gpsData.fixQuality = parseInt(token);
                break;
            case 7:
                gpsData.satellites = parseInt(token);
                break;
            case 8:
                gpsData.hdop = parseFloat(token);
                break;
            case 9:
                gpsData.altitude = parseFloat(token);
                break;
        }
        token = strtok(NULL, ",");
        fieldIndex++;
    }
    
    gpsData.validFix = (gpsData.fixQuality > 0);
    gpsData.dataUpdated = true;
    gpsData.lastUpdate = millis();
    return true;
}

bool GPSHandler::parseRMC(const char* sentence) {
    char buffer[MAX_NMEA_LENGTH];
    strncpy(buffer, sentence, sizeof(buffer));
    
    char* token = strtok(buffer, ",");
    int fieldIndex = 0;
    
    while (token != NULL) {
        switch (fieldIndex) {
            case 1: {
                if (strlen(token) >= 6) {
                    gpsData.hour = (token[0] - '0') * 10 + (token[1] - '0');
                    gpsData.minute = (token[2] - '0') * 10 + (token[3] - '0');
                    gpsData.second = (token[4] - '0') * 10 + (token[5] - '0');
                }
                break;
            }
            case 2:
                gpsData.validFix = (token[0] == 'A');
                break;
            case 3:
                if (strlen(token) > 0) {
                    gpsData.latitude = parseCoordinate(token, 'N');
                }
                break;
            case 4:
                if (token[0] == 'S') {
                    gpsData.latitude = -gpsData.latitude;
                }
                break;
            case 5:
                if (strlen(token) > 0) {
                    gpsData.longitude = parseCoordinate(token, 'E');
                }
                break;
            case 6:
                if (token[0] == 'W') {
                    gpsData.longitude = -gpsData.longitude;
                }
                break;
            case 7:
                gpsData.speed = parseFloat(token);
                break;
            case 8:
                gpsData.course = parseFloat(token);
                break;
            case 9: {
                if (strlen(token) >= 6) {
                    gpsData.day = (token[0] - '0') * 10 + (token[1] - '0');
                    gpsData.month = (token[2] - '0') * 10 + (token[3] - '0');
                    gpsData.year = (token[4] - '0') * 10 + (token[5] - '0');
                    gpsData.year += 2000;
                }
                break;
            }
        }
        token = strtok(NULL, ",");
        fieldIndex++;
    }
    
    gpsData.dataUpdated = true;
    gpsData.lastUpdate = millis();
    return true;
}

bool GPSHandler::parseGSA(const char* sentence) {
    return true;
}

bool GPSHandler::parseVTG(const char* sentence) {
    char buffer[MAX_NMEA_LENGTH];
    strncpy(buffer, sentence, sizeof(buffer));
    
    char* token = strtok(buffer, ",");
    int fieldIndex = 0;
    
    while (token != NULL) {
        switch (fieldIndex) {
            case 1:
                gpsData.course = parseFloat(token);
                break;
            case 5:
                gpsData.speed = parseFloat(token);
                break;
        }
        token = strtok(NULL, ",");
        fieldIndex++;
    }
    
    return true;
}

float GPSHandler::parseCoordinate(const char* coord, char hemisphere) {
    if (strlen(coord) == 0) return 0.0;
    
    float coordinate = atof(coord);
    
    int degrees = (int)(coordinate / 100);
    float minutes = coordinate - (degrees * 100);
    
    return degrees + (minutes / 60.0);
}

float GPSHandler::parseFloat(const char* str) {
    if (strlen(str) == 0) return 0.0;
    return atof(str);
}

int GPSHandler::parseInt(const char* str) {
    if (strlen(str) == 0) return 0;
    return atoi(str);
}

bool GPSHandler::hasValidFix() {
    return gpsData.validFix && (gpsData.fixQuality > 0);
}

GPSData GPSHandler::getGPSData() {
    return gpsData;
}

void GPSHandler::resetGPSData() {
    memset(&gpsData, 0, sizeof(gpsData));
}