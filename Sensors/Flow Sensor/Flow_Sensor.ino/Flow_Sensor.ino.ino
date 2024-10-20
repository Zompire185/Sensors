/*
Task:
Write a code that controls YF-B2 Water Flow Sensor
Scale the sensitivity as needed 
Print the information to the serial monitor after converting it to a 32 bit binary number  
00011 is the Sensor 1 ID and the remaining 27 bits for the sensor information
Digital pin connection = "2"

Specs
Min. Working Voltage DC 4.5V 
Max. Working Current 15mA (DC 5V) 
Working Voltage DC 5V~15V 
Flow Rate Range 1~25L/min 
Frequency F=(11*Q)Q=L/MIN±3% 
Load Capacity ≤10mA (DC 5V) 
Operating Temperature 0 ~ 80℃ 
Liquid Temperature ≤120℃ 
Operating Humidity 35%〜90%RH 
Water Pressure ≤1.75MPa 
Error Range (1~30L\MIN) ±3% 
Output Pulse High Level >DC 4.7V (Input Voltage DC5V) 
Output Pulse Low Level <DC 0.5V (Input Voltage DC5V) 
Output Pulse Duty Cycle 50%±10%
*/

#include <Arduino.h>

volatile int pulseCount = 0;
// Pin connected to the wire (signal) of the flow sensor
const int flowSensorPin = 2;

void setup() 
{
    pinMode(flowSensorPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, RISING);
    Serial.begin(9600);
}

void loop() {
    // Wait for 1 second
    delay(200);
    // Calibration factor
    float flowRate = (pulseCount / 4.8);
    // Convert to a 32-bit binary value

    // Add sensor ID in the first 5 bits
    uint32_t sensorData = 0b00011 << 27;  
    // Scale and fit flowRate to the next 27 bits
    sensorData |= ((uint32_t)(flowRate * 100) & 0x07FFFFFF); 
  
    // Print the sensor ID and sensor information in binary format
    Serial.print("00011");
    for (int i = 26; i >= 0; i--) 
    {
        Serial.print((sensorData >> i) & 1);
    }
    Serial.println();
    
    // Reset the pulse count
    pulseCount = 0;
}

void pulseCounter() 
{
    pulseCount++;
}
