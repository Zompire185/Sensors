/* File Header per la lettura 
dei valori dai sensori - Updated for Teensy 4.1 Integration
-------------------------
 Author: Original - Cirelli Leonardo, Updated for Integration
 Date: 21/03/2023, Updated 2025
 Version: 2.0
*/

#ifndef SENSOR_LIB_H
#define SENSOR_LIB_H

// Inclusione delle libreria da utilizzare
#include <Arduino.h>
#include <DHT.h>

// Definizione delle variabili globali dei sensori
#define SENS_CLK 4
#define SENS_ADDR 5
#define SENS_SET 34
#define SENS_READ A4

// Variabili globali del sensore di temperatura DHT
#define DHT_PIN 6
#define DHT_TYPE DHT11

/// Struttura contenente la tipologia dei sensori
enum class SENSOR_TYPE
{
	TIRE_SENS_1 = 1,	// sensore_1 gomme
	TIRE_SENS_2,		// sensore_2 gomme
	TIRE_SENS_3,		// sensore_3 gomme
	TIRE_SENS_4,		// sensore_4 gomme
	WAT_TEMP,			// sensore temperatura acqua
	AIR_TEMP_1,			// sensore_1 temperatura aria 
	AIR_TEMP_2,			// sensore_2 temperatura aria 
	AIR_TEMP_3,			// sensore_3 temperatura aria 
	WAT_PRESS,			// pressione acqua
	STEERING_SENS,		// sensore volante
	COOLING_SENS,		// sensore raffreddamento
	SUSP_SENS_1,		// sensore_1 sospensioni 
	SUSP_SENS_2,		// sensore_2 sospensioni 
	SUSP_SENS_3,		// sensore_3 sospensioni 
	SUSP_SENS_4,		// sensore_4 sospensioni 
	DIR_CURR,			// sensore trazione diretta
	INV_CURR,			// sensore trazione inversa
	BRAKE_PRESS			// sensore freni
};

/// Struttura contenente gli ID dei sensori
struct SENSOR_ID
{
	bool TIRE_SENS_1_ID[8] = {0,0,1,0,0,0,0,0};   	// id_sensore_1 gomme
	bool TIRE_SENS_2_ID[8] = {0,1,0,0,0,0,0,0}; 	  	// id_sensore_2 gomme
	bool TIRE_SENS_3_ID[8] = {0,0,0,0,0,0,0,0};   	// id_sensore_3 gomme
	bool TIRE_SENS_4_ID[8] = {0,0,1,1,0,0,0,0};   	// id_sensore_4 gomme
	bool WAT_TEMP_ID[8] = {0,0,0,1,0,0,0,0};   		// id_sensore temperatura acqua
	bool AIR_TEMP_1_ID[8] = {0,1,0,1,0,0,0,0}; 		// id_sensore_1 temperatura aria 
	bool AIR_TEMP_2_ID[8] = {0,1,1,1,0,0,0,0}; 		// id_sensore_2 temperatura aria 
	bool AIR_TEMP_3_ID[8] = {0,1,1,0,0,0,0,0};	  	// id_sensore_3 temperatura aria 
	bool WAT_PRESS_ID[8] = {1,0,0,0,0,0,0,0}; 	  	// id_pressione acqua
	bool STEERING_SENS_ID[8] = {1,0,0,1,0,0,0,0}; 	// id_sensore volante
	bool COOLING_SENS_ID[8] = {1,0,1,0,0,0,0,0};  	// id_sensore raffreddamento
	bool SUSP_SENS_1_ID[8] = {1,0,1,1,1,1,0,0};   	// id_sensore_1 sospensioni 
	bool SUSP_SENS_2_ID[8] = {1,0,1,1,1,0,0,0};   	// id_sensore_2 sospensioni
	bool SUSP_SENS_3_ID[8] = {1,0,1,1,0,1,0,0};   	// id_sensore_3 sospensioni
	bool SUSP_SENS_4_ID[8] = {1,0,1,1,0,0,0,0};   	// id_sensore_4 sospensioni
	bool DIR_CURR_ID[8] = {1,1,0,0,0,0,0,0};  	  	// id_sensore trazione diretta
	bool INV_CURR_ID[8] = {1,1,0,1,0,0,0,0};  	  	// id_sensore trazione inversa
	bool BRAKE_PRESS_ID[8] = {1,1,1,0,0,0,0,0};   	// id_sensore freni
};

/// Struttura per i dati letti dai sensori
struct SENSOR_DATA
{
	float tire_sens_1 = 0.0;	// dati sensore gomme_1
	float tire_sens_2 = 0.0; 	// dati sensore gomme_2
	float tire_sens_3 = 0.0;	// dati sensore gomme_3
	float tire_sens_4 = 0.0;	// dati sensore gomme_4
	float air_temp_1 = 0.0;		// dati sensore temperatura aria_1
	float air_temp_2 = 0.0;		// dati sensore temperatura aria_2
	float air_temp_3 = 0.0;		// dati sensore temperatura aria_3
	float water_temp = 0.0;		// dati sensore temperatura acqua
	float water_press = 0.0;	// dati sensore pressione acqua
	float steering_data = 0.0;	// dati sensore volante
	float cooling_data = 0.0;	// dati sensore raffreddamento
	float susp_data_1 = 0.0;	// dati sensore sospensioni_1
	float susp_data_2 = 0.0;	// dati sensore sospensioni_2
	float susp_data_3 = 0.0;	// dati sensore sospensioni_3
	float susp_data_4 = 0.0;	// dati sensore sospensioni_4
	float dir_curr_data = 0.0;	// dati sensore APPS_Diretto
	float inv_curr_data = 0.0;	// dati sensore APPS_Inverso
	float brake_data = 0.0;		// dati sensore freni
	float Temperature = 0.0;	// dati sensore temperatura motherboard
	float Humidity = 0.0;		// dati sensore umidità motherboard
};

class SENSOR
{
	public:
		// Struttura per i dati letti dai sensori
		SENSOR_DATA Sensor_Data;
		
		// Constructor - DECLARED HERE
		SENSOR();
		
		// Destructor - DECLARED HERE
		~SENSOR();
		
		// Public methods - DECLARED HERE
		bool Sensor_begin(bool *start_sensor, uint8_t *Error_Dashboard, bool *Error_Free, bool* Buffer_Led);
		void Tx_Addr(bool* sens_addr);
		float Get_Data(SENSOR_TYPE sensor);
		void Motherboard_Temp(bool* Buffer_Led, uint8_t *Error_Buffer);
		void Check_Sensor(SENSOR_TYPE sensor, float sens_value, uint8_t *Error_Dashboard, bool *Buffer_Led);
		void APPS_Manager(void);
		
	private:
		// Private variables
		int count;
		SENSOR_ID Sensor_ID;
		DHT* dht;
		
		// Private methods - DECLARED HERE
		float calculateNTCTemperature(float adcValue, float beta);
};

#endif