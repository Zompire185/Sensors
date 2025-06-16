/* File della definizione dei metodi 
   per la lettura dei valori dai sensori - Updated for Teensy 4.1
   e per la gestione del sistema APPS
--------------------------
 Author: Original - Cirelli Leonardo, Updated for Integration
 Date: 21/03/2023, Updated 2025
 Version: 2.0
*/

#include "sensor_lib.h"

// Constructor
SENSOR::SENSOR() {
    dht = new DHT(DHT_PIN, DHT_TYPE);
    count = 0;
}

// Destructor
SENSOR::~SENSOR() {
    delete dht;
}

/// @brief Metodo di inizializzazione dei sensori e dei canali di comunicazione
/// @param start_sensor variabile di inizializzazione dei sensori attesa dall'interrupt
bool SENSOR::Sensor_begin(bool *start_sensor, uint8_t *Error_Dashboard, bool *Error_Free, bool* Buffer_Led)
{
	// Canali di comunicazione in scrittura
	pinMode(SENS_CLK, OUTPUT);
	pinMode(SENS_ADDR, OUTPUT);
	
	// Canali di comunicazione in lettura
	pinMode(SENS_READ, INPUT);
	pinMode(DHT_PIN, INPUT);
	pinMode(SENS_SET, INPUT);
	
	// Inizializzazione del sensore DHT
	dht->begin();
	
	digitalWrite(SENS_CLK, HIGH);
	delay(10);
	digitalWrite(SENS_ADDR, LOW);

	count = 0;
	
	while (!(*start_sensor) && count < 16)
	{
		digitalWrite(SENS_CLK, LOW);
		delay(10);
		digitalWrite(SENS_CLK, HIGH);
		count++;
	}
	
	if (!(*start_sensor))
	{
		// Comunicazione Errore alla Dashboard (Setup_Sensor)
		Error_Dashboard[3] |= 0b00000001;
		*Error_Free = false;	
		Buffer_Led[13] = false; // SENS_OK = OFF
		return false;
	} 
	else 
	{
		*Error_Free = true;
		// Reset Errore alla Dashboard (Setup_Sensor)
		Error_Dashboard[3] &= 0b11111110;
		Buffer_Led[13] = true; // SENS_OK = ON
	}
	
	return *Error_Free;
}

float SENSOR::calculateNTCTemperature(float adcValue, float beta) {
    float resistance = 10000.0 * ((16383.0 / adcValue) - 1.0);
    float temperature = (beta * 298.15) / (beta + 298.15 * log(resistance / 10000.0)) - 273.15;
    return temperature;
}

/// @brief Metodo di lettura dei dati del sensore
/// @param sensor Sensore da cui prendere i dati
float SENSOR::Get_Data(SENSOR_TYPE sensor)
{
	float S_value = 0;
	
	if(sensor == SENSOR_TYPE::TIRE_SENS_1) // Sensore: ZTP-148SRC1
	{
		Tx_Addr(Sensor_ID.TIRE_SENS_1_ID);	
		S_value = (analogRead(SENS_READ)*0.0073) - 20.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::TIRE_SENS_2)
	{
		Tx_Addr(Sensor_ID.TIRE_SENS_2_ID);
		S_value = (analogRead(SENS_READ)*0.0073) - 20.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::TIRE_SENS_3)
	{
		Tx_Addr(Sensor_ID.TIRE_SENS_3_ID);
		S_value = (analogRead(SENS_READ)*0.0073) - 20.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::TIRE_SENS_4)
	{
		Tx_Addr(Sensor_ID.TIRE_SENS_4_ID);
		S_value = (analogRead(SENS_READ)*0.0073) - 20.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::WAT_TEMP) // Sensore: PANW103395-395 (Termistore AQ)
	{
		Tx_Addr(Sensor_ID.WAT_TEMP_ID);
		float adcValue = analogRead(SENS_READ);
		S_value = calculateNTCTemperature(adcValue, 3950.0);
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::AIR_TEMP_1) // Sensore: 2152721603 (Termistore AR)
	{
		Tx_Addr(Sensor_ID.AIR_TEMP_1_ID);
		float adcValue = analogRead(SENS_READ);
		S_value = calculateNTCTemperature(adcValue, 3892.0);
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::AIR_TEMP_2)
	{
		Tx_Addr(Sensor_ID.AIR_TEMP_2_ID);
		float adcValue = analogRead(SENS_READ);
		S_value = calculateNTCTemperature(adcValue, 3892.0);
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::AIR_TEMP_3)
	{
		Tx_Addr(Sensor_ID.AIR_TEMP_3_ID);
		float adcValue = analogRead(SENS_READ);
		S_value = calculateNTCTemperature(adcValue, 3892.0);
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::WAT_PRESS) // Water Flow converted to pressure
	{
		Tx_Addr(Sensor_ID.WAT_PRESS_ID); 
		S_value = (analogRead(SENS_READ) * 25.0) / 16383.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::STEERING_SENS)
	{
		Tx_Addr(Sensor_ID.STEERING_SENS_ID);
		float adcValue = analogRead(SENS_READ);
		S_value = ((adcValue / 16383.0) - 0.5) * 360.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::COOLING_SENS) // Sensore: MIPAN2XX100PSAAX
	{
		Tx_Addr(Sensor_ID.COOLING_SENS_ID); 
		S_value = (analogRead(SENS_READ) * 100.0) / 16383.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_1) // Potenziometri Lineari
	{
		Tx_Addr(Sensor_ID.SUSP_SENS_1_ID);
		S_value = (analogRead(SENS_READ) * 75.0) / 16383.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_2)
	{
		Tx_Addr(Sensor_ID.SUSP_SENS_2_ID);
		S_value = (analogRead(SENS_READ) * 75.0) / 16383.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_3)
	{
		Tx_Addr(Sensor_ID.SUSP_SENS_3_ID);
		S_value = (analogRead(SENS_READ) * 75.0) / 16383.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_4)
	{
		Tx_Addr(Sensor_ID.SUSP_SENS_4_ID);
		S_value = (analogRead(SENS_READ) * 75.0) / 16383.0;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::DIR_CURR) // Sensore: MT7190-ND (100A current sensor)
	{
		Tx_Addr(Sensor_ID.DIR_CURR_ID);
		float adcValue = analogRead(SENS_READ);
		float voltage = (adcValue * 3.3) / 16383.0;
		S_value = (voltage - 2.5) / 0.04;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::INV_CURR)
	{
		Tx_Addr(Sensor_ID.INV_CURR_ID);
		float adcValue = analogRead(SENS_READ);
		float voltage = (adcValue * 3.3) / 16383.0;
		S_value = (voltage - 2.5) / 0.04;
		return S_value;
	}
	else if(sensor == SENSOR_TYPE::BRAKE_PRESS) // Sensore: MIPAF1XX667PSAAX
	{
		Tx_Addr(Sensor_ID.BRAKE_PRESS_ID); 
		float adcValue = analogRead(SENS_READ);
		float voltage = (adcValue * 3.3) / 16383.0;
		
		if (voltage >= 0.5 && voltage <= 4.5) {
			S_value = ((voltage - 0.5) / 4.0) * 667.0;
		} else {
			S_value = 0.0;
		}
		return S_value;
	}
	else return 0;
}

/// @brief Metodo di lettura della temperatura della scheda madre (DHT11)
/// @param Buffer_Led Buffer dei led di stato
/// @param Error_Buffer Buffer degli errori alla dashboard
void SENSOR::Motherboard_Temp(bool* Buffer_Led, uint8_t *Error_Buffer)
{
	Sensor_Data.Temperature = dht->readTemperature();
	Sensor_Data.Humidity = dht->readHumidity();
	
	if(isnan(Sensor_Data.Temperature) || isnan(Sensor_Data.Humidity))
	{
		Error_Buffer[4] |= 0b10000000;
		if(Buffer_Led[13])
		{Buffer_Led[13] = false;}
	} 
	else 
	{
		Error_Buffer[4] &= 0b01111111;
		if(!Buffer_Led[13])
		{Buffer_Led[13] = true;}
	}
}

/// @brief Metodo di invio degli indirizzi dei sensori
///@param sens_addr Buffer contenente gli indirizzi dei sensori
void SENSOR::Tx_Addr(bool* sens_addr)
{
	digitalWrite(SENS_CLK, HIGH);
	digitalWrite(SENS_ADDR, LOW);
	
	for(int i = 7; i >= 0; i--)
	{	
		digitalWrite(SENS_CLK, LOW);

		if(sens_addr[i] == 0)
			digitalWrite(SENS_ADDR, LOW);
		else if (sens_addr[i] == 1)
			digitalWrite(SENS_ADDR, HIGH);

		digitalWrite(SENS_CLK, HIGH);
	}
}

/// Metodo per il check della correttezza dati dei sensori
void SENSOR::Check_Sensor(SENSOR_TYPE sensor, float sens_value, uint8_t *Error_Dashboard, bool *Buffer_Led)
{
	if(sensor == SENSOR_TYPE::TIRE_SENS_1)
	{
		if(sens_value <= -25.0 || sens_value >= 120.0)
		{
			Error_Dashboard[2] |= 0b10000000; 
			Buffer_Led[13] = false;
			Buffer_Led[6] = true;
		}
		else 
		{
			Error_Dashboard[2] &= 0b01111111; 
			Buffer_Led[13] = true; 
			Buffer_Led[6] = false; 
		}
	}
	else if(sensor == SENSOR_TYPE::TIRE_SENS_2)
	{
		if(sens_value <= -25.0 || sens_value >= 120.0)
		{
			Error_Dashboard[2] |= 0b01000000; 
			Buffer_Led[13] = false;
			Buffer_Led[6] = true;
		}
		else 
		{
			Error_Dashboard[2] &= 0b10111111; 
			Buffer_Led[13] = true; 
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::TIRE_SENS_3)
	{
		if(sens_value <= -25.0 || sens_value >= 120.0)
		{
			Error_Dashboard[2] |= 0b00100000; 
			Buffer_Led[13] = false;
			Buffer_Led[6] = true;
		}
		else 
		{
			Error_Dashboard[2] &= 0b11011111; 
			Buffer_Led[13] = true; 
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::TIRE_SENS_4)
	{
		if(sens_value <= -25.0 || sens_value >= 120.0)
		{
			Error_Dashboard[2] |= 0b00010000; 
			Buffer_Led[13] = false;
			Buffer_Led[6] = true;
		}
		else 
		{
			Error_Dashboard[2] &= 0b11101111;
			Buffer_Led[13] = true; 
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::AIR_TEMP_1)
	{
		if(sens_value <= -42.0 || sens_value >= 130.0)
		{
			Error_Dashboard[1] |= 0b00000100; 
			Buffer_Led[13] = false;
			Buffer_Led[7] = false;
			Buffer_Led[6] = true;
		}
		else 
		{
			Error_Dashboard[1] &= 0b11111011; 
			Buffer_Led[13] = true; 
			Buffer_Led[7] = true;
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::AIR_TEMP_2)
	{
		if(sens_value <= -42.0 || sens_value >= 137.0)
		{
			Error_Dashboard[1] |= 0b00000010; 
			Buffer_Led[13] = false;
			Buffer_Led[7] = false;
			Buffer_Led[6] = true;
		}
		else 
		{
			Error_Dashboard[1] &= 0b11111101;
			Buffer_Led[13] = true; 
			Buffer_Led[7] = true;
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::AIR_TEMP_3)
	{
		if(sens_value <= -42.0 || sens_value >= 137.0)
		{
			Error_Dashboard[1] |= 0b00000001; 
			Buffer_Led[13] = false;
			Buffer_Led[7] = false;
			Buffer_Led[6] = true;
		}
		else 
		{
			Error_Dashboard[1] &= 0b11111110;
			Buffer_Led[13] = true; 
			Buffer_Led[7] = true;
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_1)
	{
		if(sens_value < 0.0 || sens_value > 75.0)
		{
			Error_Dashboard[1] |= 0b10000000; 
			Buffer_Led[13] = false;
		}
		else 
		{
			Error_Dashboard[1] &= 0b01111111; 
			Buffer_Led[13] = true; 
		}
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_2)
	{
		if(sens_value < 0.0 || sens_value > 75.0)
		{
			Error_Dashboard[1] |= 0b01000000; 
			Buffer_Led[13] = false;
		}
		else 
		{
			Error_Dashboard[1] &= 0b10111111; 
			Buffer_Led[13] = true; 
		}
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_3)
	{
		if(sens_value < 0.0 || sens_value > 75.0)
		{
			Error_Dashboard[1] |= 0b00100000; 
			Buffer_Led[13] = false;
		}
		else 
		{
			Error_Dashboard[1] &= 0b11011111; 
			Buffer_Led[13] = true; 
		}
	}
	else if(sensor == SENSOR_TYPE::SUSP_SENS_4)
	{
		if(sens_value < 0.0 || sens_value > 75.0)
		{
			Error_Dashboard[1] |= 0b00010000; 
			Buffer_Led[13] = false;
		}
		else 
		{
			Error_Dashboard[1] &= 0b11101111; 
			Buffer_Led[13] = true; 
		}
	}
	else if(sensor == SENSOR_TYPE::DIR_CURR)
	{
		if(abs(sens_value) > 100.0)
		{
			Error_Dashboard[3] |= 0b10000000; 
			Buffer_Led[13] = false;
			Buffer_Led[6] = true;
		}
		else
		{
			Error_Dashboard[3] &= 0b01111111; 
			Buffer_Led[13] = true; 
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::INV_CURR)
	{
		if(abs(sens_value) > 100.0) 
		{	
			Error_Dashboard[3] |= 0b01000000; 
			Buffer_Led[13] = false;
			Buffer_Led[6] = true;
		}
		else
		{
			Error_Dashboard[3] &= 0b10111111; 
			Buffer_Led[13] = true; 
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::WAT_TEMP)
	{
		if(sens_value <= -52.0 || sens_value >= 152.0)
		{
			Error_Dashboard[3] |= 0b00010000; 
			Buffer_Led[13] = false;
			Buffer_Led[7] = false;
			Buffer_Led[6] = true;
		}
		else
		{
			Error_Dashboard[3] &= 0b11101111; 
			Buffer_Led[13] = true; 
			Buffer_Led[7] = true;
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::BRAKE_PRESS)
	{
		if(sens_value >= 670.0)
		{
			Error_Dashboard[3] |= 0b00100000; 
			Buffer_Led[13] = false;
			Buffer_Led[6] = true;
		}
		else
		{
			Error_Dashboard[3] &= 0b11011111; 
			Buffer_Led[13] = true; 
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::WAT_PRESS)
	{
		if(sens_value <= 0.5 || sens_value >= 27.0)
		{
			Error_Dashboard[3] |= 0b00001000; 
			Buffer_Led[13] = false;
			Buffer_Led[7] = false;
			Buffer_Led[6] = true;
		}
		else
		{
			Error_Dashboard[3] &= 0b11110111; 
			Buffer_Led[13] = true; 
			Buffer_Led[7] = true;
			Buffer_Led[6] = false;
		}
	}
	else if(sensor == SENSOR_TYPE::STEERING_SENS)
	{
		if(abs(sens_value) > 180.0)
		{
			Error_Dashboard[3] |= 0b00000100; 
			Buffer_Led[13] = false;
		}
		else
		{
			Error_Dashboard[3] &= 0b11111011; 
			Buffer_Led[13] = true; 
		}
	}
	else if(sensor == SENSOR_TYPE::COOLING_SENS)
	{
		if(sens_value <= 0.0 || sens_value >= 102.0)
		{
			Error_Dashboard[3] |= 0b00000010; 
			Buffer_Led[13] = false;
			Buffer_Led[7] = false;
			Buffer_Led[6] = true;
		}
		else
		{
			Error_Dashboard[3] &= 0b11111101; 
			Buffer_Led[13] = true; 
			Buffer_Led[7] = true;
			Buffer_Led[6] = false;
		}
	}
}

/// Metodo di gestione APPS
void SENSOR::APPS_Manager(void)
{
	// Lettura dei dati critici per APPS
	Sensor_Data.brake_data = Get_Data(SENSOR_TYPE::BRAKE_PRESS);
	Sensor_Data.dir_curr_data = Get_Data(SENSOR_TYPE::DIR_CURR);
	
	// Aggiorna i dati dal sistema di sospensioni per monitoraggio
	Sensor_Data.susp_data_1 = Get_Data(SENSOR_TYPE::SUSP_SENS_1);
	Sensor_Data.susp_data_2 = Get_Data(SENSOR_TYPE::SUSP_SENS_2);
	Sensor_Data.susp_data_3 = Get_Data(SENSOR_TYPE::SUSP_SENS_3);
	Sensor_Data.susp_data_4 = Get_Data(SENSOR_TYPE::SUSP_SENS_4);
	
	// Aggiorna i dati delle temperature
	Sensor_Data.tire_sens_1 = Get_Data(SENSOR_TYPE::TIRE_SENS_1);
	Sensor_Data.tire_sens_2 = Get_Data(SENSOR_TYPE::TIRE_SENS_2);
	Sensor_Data.tire_sens_3 = Get_Data(SENSOR_TYPE::TIRE_SENS_3);
	Sensor_Data.tire_sens_4 = Get_Data(SENSOR_TYPE::TIRE_SENS_4);
	
	Sensor_Data.air_temp_1 = Get_Data(SENSOR_TYPE::AIR_TEMP_1);
	Sensor_Data.air_temp_2 = Get_Data(SENSOR_TYPE::AIR_TEMP_2);
	Sensor_Data.air_temp_3 = Get_Data(SENSOR_TYPE::AIR_TEMP_3);
	
	Sensor_Data.water_temp = Get_Data(SENSOR_TYPE::WAT_TEMP);
	Sensor_Data.water_press = Get_Data(SENSOR_TYPE::WAT_PRESS);
	Sensor_Data.steering_data = Get_Data(SENSOR_TYPE::STEERING_SENS);
	Sensor_Data.cooling_data = Get_Data(SENSOR_TYPE::COOLING_SENS);
	Sensor_Data.inv_curr_data = Get_Data(SENSOR_TYPE::INV_CURR);
	
	// Logica di sicurezza APPS
	// Se il pedale del freno è premuto oltre una soglia e viene rilevato 
	// un comando di accelerazione, si genera un errore di plausibilità
	if (Sensor_Data.brake_data > 50.0) { // Soglia freno in psi
		// In caso di sistema APPS completo, qui si controllerebbe 
		// anche la posizione dell'acceleratore
		// Per ora monitoriamo solo i valori critici
	}
	
	// Controllo limiti di corrente per sicurezza
	if (abs(Sensor_Data.dir_curr_data) > 90.0 || abs(Sensor_Data.inv_curr_data) > 90.0) {
		// Condizione di overcurrent - dovrebbe attivare protezioni
	}
}