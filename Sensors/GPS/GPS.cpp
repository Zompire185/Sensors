/* 
Task:
-Modify the GPS parsing code already present on the Adafruit website to our needs 
-Change the output on the serial monitor to a 32-bit binary number 
-10010 should be the first 5 bits which are the sensor id and the remaining 27 bits should  containing the Date, time
-Latitude with id 01111 and remaining 27 for the data 
-Longitude with id 10000 and remaining 27 for the data, 
-Speed with 10100 and remaining 27 for the data, 
-GPS Angle 10101 and remaining 27 for the data, 
-GPS Fix & Quality 10110 and remaining 27 for the data
*/

#include <Arduino.h>
#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

// Function to convert integer to binary string of a specific length
String toBinaryString(uint32_t num, int length) 
{
  String binStr = "";
  for (int i = length - 1; i >= 0; i--) 
  {
    binStr += (num >> i) & 1 ? "1" : "0";
  }
  return binStr;
}

void setup() 
{
  // Start Serial communication for debugging
  Serial.begin(9600);

  // Initialize the GPS module
  GPS.begin(0x10);  // The I2C address to use is 0x10

  // Configure the GPS module to output RMC (recommended minimum) and GGA (fix data)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate to 1 Hz (1 update per second)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Request updates on antenna status (optional)
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Query the GPS firmware version
  GPS.println(PMTK_Q_RELEASE);
}

void loop() 
{
  // Read data from the GPS
  char c = GPS.read();

  // Echo raw data to Serial if GPSECHO is enabled (disabled by default)
  if (GPSECHO)
    if (c) Serial.print(c);

  // Check if a new NMEA sentence has been received
  if (GPS.newNMEAreceived()) 
  {
    // Parse the new NMEA sentence
    if (!GPS.parse(GPS.lastNMEA()))
      // If parsing fails, wait for another sentence
      return; 
  }

  // Output date, time, location, and speed approximately every second
  if (millis() - timer > 1000) 
  {
    // Reset the timer
    timer = millis(); 

    if (GPS.fix) 
    {
      // Date 
      uint32_t dateData = (GPS.year * 10000) + (GPS.month * 100) + GPS.day;
      uint16_t dateBinary = dateData % (1 << 14); // Mod to fit into 14 bits
      String dateBinaryStr = toBinaryString(dateBinary, 14);

      // Time 
      uint32_t timeData = (GPS.hour * 3600) + (GPS.minute * 60) + GPS.seconds;
      // Scale to fit into 15 bits
      uint16_t timeBinary = timeData % (1 << 15); 
      String timeBinaryStr = toBinaryString(timeBinary, 15);

      // Combine sensor ID with date and time
      String dateTimeBinary = "10010" + dateBinaryStr + timeBinaryStr;
      Serial.println(dateTimeBinary);

      // Latitude
      // Scaling to fit within 27 bits
      uint32_t latitudeData = (uint32_t)(GPS.latitude * 10000); 
      Serial.println("01111" + toBinaryString(latitudeData, 27));

      // Longitude
      // Scaling to fit within 27 bits
      uint32_t longitudeData = (uint32_t)(GPS.longitude * 10000); 
      Serial.println("10000" + toBinaryString(longitudeData, 27));

      // Speed (convert from knots to km/h)
      // Convert speed from knots to km/h
      float speed_kmh = GPS.speed * 1.852;  
      // Scaling to fit within 27 bits
      uint32_t speedData = (uint32_t)(speed_kmh * 100); 
      Serial.println("10100" + toBinaryString(speedData, 27));

      // GPS Angle
      // Scaling to fit within 27 bits
      uint32_t angleData = (uint32_t)(GPS.angle * 100); 
      Serial.println("10101" + toBinaryString(angleData, 27));

      // GPS Fix & Quality
      uint32_t fixQualityData = (GPS.fixquality << 24) | (GPS.satellites << 16) | (GPS.fix);
      Serial.println("10110" + toBinaryString(fixQualityData, 27));
    } 
    else 
    {
      Serial.println("Location: No GPS fix");
    }

    // Print a newline for readability
    Serial.println(); 
  }
}




