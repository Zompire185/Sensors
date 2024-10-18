
/* Task
 Write a code that controls 2 PANW 103395-395 NTC Thermistor probes
 Will be using a 10KΩ fixed resistor for both probes 
 Scale the sensitivity as needed 
 Print the information to the serial monitor after converting it to a 32 bit binary number  
 00010 is the sensor 1 ID and the remaining 27 bits for the sensor information
 11110 is the sensor 2 ID and the remaining 27 bits for the sensor information 
  Electrical specifications 
   Resistance @ 25 C 10 kΩ ±3% 
   Temperature Coefficient of Resistance -4.43% / ºC 
   Operating Temperature Range -50 ºC to 150 ºC 
   Dissipation Constant 8 mW / ºC 
   Thermal Time Constant 10 seconds 
   Material Constant (Beta) 3950 ºK ±2% 
   ROHS Compliant Yes 
   MSL (moisture sensitivity level) 1 
Analog Port = A0 for sensor 1
Analog Port = A1 for sensor 2
*/

#include <Arduino.h> // Include the Arduino library for basic functions

// Constants 

// Define the analog input pin for the first thermistor
const int thermistorPin1 = A0; 

// Define the analog input pin for the second thermistor
const int thermistorPin2 = A1; 

// Define the value of the fixed resistor (10K ohm)
const int fixedResistor = 10000; 

// Define the Beta value for the thermistor
const float beta = 3950.0; 

// Define a 5-bit sensor ID for sensor 1
const int sensorID1 = 0b00010; 

// Define a 5-bit sensor ID for sensor 2
const int sensorID2 = 0b11110; 

// Define nominal resistance at 25 degrees Celsius (10K ohms)
const float nominalResistance = 10000.0; 

// Define nominal temperature (25 degrees Celsius)
const float nominalTemperature = 25.0; 

// Define maximum ADC value for a 10-bit ADC
const float adcMaxValue = 1023.0; 

// Define the voltage of the series circuit
const float Voltage = 5.0; 

void setup() 
{
    // Initialize serial communication at a baud rate of 9600
    Serial.begin(9600); 
}

void loop() 
{
    // Read and print data from the first thermistor
    readAndPrintSensor(thermistorPin1, sensorID1); 
    
    // Read and print data from the second thermistor
    readAndPrintSensor(thermistorPin2, sensorID2); 
    
    // Wait for 1 second before taking the next readings
    delay(1000); 
}

// Function to read temperature from a thermistor and print its data
void readAndPrintSensor(int thermistorPin, int sensorID) 
{
    // Read the analog value from the specified thermistor pin
    int adcValue = analogRead(thermistorPin); 
    
    // Convert ADC value to voltage using the series voltage and max ADC value
    float voltage = (adcValue / adcMaxValue) * Voltage; 
    
    // Calculate thermistor resistance using voltage divider formula
    float resistance = (Voltage / voltage - 1) * fixedResistor; 

    // Calculate temperature in Celsius
    float temperature = (1.0 / ((1.0 / (nominalTemperature + 273.15)) + (1.0 / beta) * log(resistance / nominalResistance))) - 273.15; // Convert Kelvin to Celsius

    // Scale temperature from -50 to +150 degrees Celsius to fit into a 27-bit representation
    long scaledTemperature = (long)((temperature + 50) * 1000); 

    // Pack the information into a single 32-bit binary number
    unsigned long sensorData = (sensorID << 27) | (scaledTemperature & 0x07FFFFFF); 
    
    // Print both sensor ID and sensor data in binary format on the same line
    Serial.print(sensorID, BIN); // Print sensor ID in binary format
    
    // Loop through each bit of sensorData and print it in binary format
    for (int i = 31; i >= 0; i--) 
    { 
        Serial.print((sensorData >> i) & 1); // Print each bit of sensorData
    }
    
    // Move to next line after printing all bits
    Serial.println(); 
}