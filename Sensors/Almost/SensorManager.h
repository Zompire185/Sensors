#ifndef GPSHANDLER_H
#define GPSHANDLER_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// GPS pin definitions
#define GPS_RX_PIN 0
#define GPS_TX_PIN 1
#define GPS_ENABLE_PIN 2
#define GPS_3D_FIX_PIN 5

// GPS constants
#define GPS_BAUD_RATE 9600
#define GPS_BUFFER_SIZE 256
#define MAX_NMEA_LENGTH 128

struct GPSData {
    // Date and time
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    
    // Position
    float latitude;
    float longitude;
    float altitude;
    
    // Motion
    float speed;
    float course;
    
    // Quality indicators
    uint8_t fixQuality;
    uint8_t satellites;
    float hdop;
    
    // Status
    bool validFix;
    bool dataUpdated;
    
    // Timestamp
    unsigned long lastUpdate;
};

class GPSHandler {
private:
    SoftwareSerial* gpsSerial;
    GPSData gpsData;
    char nmeaBuffer[GPS_BUFFER_SIZE];
    uint16_t bufferIndex;
    
    // NMEA parsing methods - DECLARED HERE
    bool parseGGA(const char* sentence);
    bool parseRMC(const char* sentence);
    bool parseGSA(const char* sentence);
    bool parseVTG(const char* sentence);
    
    // Utility methods - DECLARED HERE
    float parseCoordinate(const char* coord, char hemisphere);
    float parseFloat(const char* str);
    int parseInt(const char* str);
    bool validateChecksum(const char* sentence);
    void resetGPSData();
    
public:
    // Constructor
    GPSHandler();
    
    // Destructor
    ~GPSHandler();
    
    // Public methods - DECLARED HERE
    bool begin();
    void update();
    bool hasValidF