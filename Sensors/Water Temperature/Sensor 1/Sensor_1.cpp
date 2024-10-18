/* Task
 Write a code that controls the PANW 103395-395 NTC Thermistor probe
 Will be using a 10KΩ fixed resistor 
 Scale the sensitivity as needed 
 Print the information to the serial monitor after converting it to a 32 bit binary number  
 00010 is the sensor ID and the remaining 27 bits for the sensor information 
  Electrical specifications 
   Resistance @ 25 C 10 kΩ ±3% 
   Temperature Coefficient of Resistance -4.43% / ºC 
   Operating Temperature Range -50 ºC to 150 ºC 
   Dissipation Constant 8 mW / ºC 
   Thermal Time Constant 10 seconds 
   Material Constant (Beta) 3950 ºK ±2% 
   ROHS Compliant Yes 
   MSL (moisture sensitivity level) 1 
Analog Port = A0
*/

// Include the Arduino library for basic functions
#include <Arduino.h> 


// Constants
  // Define the analog input pin for the thermistor
  const int thermistorPin = A0; 

  // Define the value of the fixed resistor (10K ohm)
  const int fixedResistor = 10000; 

  // Define the Beta value for the thermistor
  const float beta = 3950.0; 

  // Define a 5-bit sensor ID (binary representation)
  const int sensorID = 0b00010; 

  // Define nominal resistance at 25 degrees Celsius (10K ohms)
  const float nominalResistance = 10000.0; 

  // Define nominal temperature (25 degrees Celsius)
  const float nominalTemperature = 25.0; 

  // Define maximum ADC value for a 10-bit ADC
  const float adcMaxValue = 1023.0; 

  // Define the voltage of the  circuit
  const float Voltage = 5.0; 

void setup() 
{
    // Initialize serial communication at a baud rate of 9600
    Serial.begin(9600); 
}

void loop() 
{
    // Read the analog value from the thermistor pin
    int adcValue = analogRead(thermistorPin); 
    
    // Convert ADC value to voltage
    float voltage = (adcValue / adcMaxValue) * Voltage; 
    
    // Calculate thermistor resistance using voltage divider formula
    float resistance = (Voltage / voltage - 1) * fixedResistor; 

    // Calculate temperature in Celsius 
    // Convert Kelvin to Celsius
    float temperature = (1.0 / ((1.0 / (nominalTemperature + 273.15)) + (1.0 / beta) * log(resistance / nominalResistance))) - 273.15; 

    // Scale temperature from -50 to +150 degrees Celsius
    long scaledTemperature = (long)((temperature + 50) * 1000); 

    // Combine sensor ID and scaled temperature into a single 32-bit binary number
    unsigned long sensorData = (sensorID << 27) | (scaledTemperature & 0x07FFFFFF); 

    // Print sensor ID in binary format
    Serial.print(sensorID, BIN); 
    
    // Loop through each bit of scaledTemperature and print it
    for (int i = 26; i >= 0; i--) 
    { 
        // Print each bit of scaledTemperature
        Serial.print((scaledTemperature >> i) & 1); 
    }
    
    // Move to next line after printing all bits
    Serial.println(); 

    // Wait for 1 second before taking another reading
    delay(1000); 
}