/*
task: 
Obtain data from an accelerometer on the X and Y Axis  
Print to serial monitor after converting it to a 32 bit binary number.
Specs
The accelerometer is a Adafruit LIS3DH Triple-Axis Accelerometer
Connected through I2C.
Set accelerometer precision to 4G
10001 is the sensor id and the remaining 27 bits should contain the sensor data 

*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// Function to convert an integer to a binary string with leading zeros
String toBinaryString(uint32_t num, int bits) 
{
    String binary = "";
    for (int i = bits - 1; i >= 0; i--) 
      {
        binary += ((num >> i) & 1) ? '1' : '0';
      }
    return binary;
}

void setup() 
  {
    Serial.begin(115200);
    
    // Initialize the LIS3DH
    if (!lis.begin(0x18)) 
      {  // Change to 0x19 for alternative I2C address
        Serial.println("Could not find a valid LIS3DH sensor, check wiring!");
        while (1);
      }
    
    // Set accelerometer precision to 4G
    lis.setRange(LIS3DH_RANGE_4_G);
  }

void loop() 
  {
    sensors_event_t event; 
    lis.getEvent(&event);
    
    // Get X and Y axis data

    // Convert to milli-g
    int16_t x = event.acceleration.x * 1000; 
    // Convert to milli-g
    int16_t y = event.acceleration.y * 1000; 

    // Scale X-axis to a 15-bit number

    // Masking to get last 15 bits
    uint16_t x_scaled = (uint16_t)(x & 0x7FFF); 

    // Scale Y-axis to a 14-bit number

    // Masking to get last 14 bits
    uint16_t y_scaled = (uint16_t)(y & 0x3FFF); 

    // Print the complete 32-bit binary output
    
    // Sensor ID (5 bits)
    String output = "10001"; 
    
    // Print X-axis as a 15-bit binary number with leading zeros
    output += toBinaryString(x_scaled, 15); 
    
    // Print Y-axis as a 14-bit binary number with leading zeros
    output += toBinaryString(y_scaled, 14); 
    
    // Print the final 32-bit string
    Serial.println(output);
    
    // Delay for readability
    delay(1000);
}
