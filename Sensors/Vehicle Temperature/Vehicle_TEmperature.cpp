/* 
Task 
Obtain Information from a DHT11 temperature and humidity sensor 
Print the information to the serial monitor after converting it to a 32 bit binary number  
Specs
01101 Temperature ID remaining 27 bits for the information
01110 Humidity ID remaining 27 bits for the information 
*/
#include <Arduino.h>
#include <DHT.h>


// Pin connected to the DHT sensor
#define DHTPIN 8      
// DHT 11 sensor type
#define DHTTYPE DHT11  


DHT dht(DHTPIN, DHTTYPE);


void setup()
{
  // Initialize serial communication
  Serial.begin(9600);
  // Initialize DHT sensor
  dht.begin();        
}


void loop()
{
  // Read temperature and humidity from the DHT11 sensor
  int temperature = dht.readTemperature();
  int humidity = dht.readHumidity();


  // Ensure valid data is received
  if (isnan(temperature) || isnan(humidity))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Convert temperature and humidity to 27-bit values

  // Limit the data to 27 bits for each ID

  // Mask the temperature value to fit in 27 bits
  unsigned long tempData = (unsigned long)temperature & 0x07FFFFFF;
  // Mask the humidity value to fit in 27 bits
  unsigned long humidData = (unsigned long)humidity & 0x07FFFFFF;    
 
  // Create the binary format (ID + data)
 
  // Shift ID and combine with temperature data
  unsigned long tempBinary = (0b01101L << 27) | tempData;  
  // Shift ID and combine with humidity data
  unsigned long humidBinary = (0b01110L << 27) | humidData;
 
  // Print the data in 32-bit binary format

  // Print temperature as binary
  Serial.println(tempBinary, BIN);
 
  // Print humidity as binary
  Serial.println(humidBinary, BIN);
 
  // Wait for 2 seconds before next reading
  delay(2000);
  
}