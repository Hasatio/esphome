#pragma once

#include "mux.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_EC10.h"
#include "DFRobot_PH.h"

namespace esphome {
namespace water_quality {

static const char *const analog = "analog";

static unsigned long timepoint = millis();
unsigned long intervals[] = {
	1000U,      //0
	2000U,	    //1
	3000U,	    //2
	5000U,      //3
	10000U,     //4
	15000U,     //5
	20000U,     //6
	25000U,     //7
	60000U,     //8
	1800000U,   //9
};			    //this defines the interval for each task in milliseconds
unsigned long last[10] = {0};

uint16_t adc[8], AnInWT_Res = 1000; //temperature sensor model pt1000 and its resistance is 1k
float volts[8], WT_Res, WT, VPow, LvlPerc[2], AnGen[2];
float  voltagePH, voltageEC, phValue, ecValue, lastTemperature;
char cmd[10];
uint8_t top, AnInGen_Ch[2], rnd;

float ecVoltage,phVoltage,temperature;

std::vector<uint16_t> AnInLvl_ResMin{0};
std::vector<uint16_t> AnInLvl_ResMax{0};
uint8_t AnInEC_Ch, AnInEC_Type, AnInPH_Ch, AnInPH_Type;

class ADS1115
{
public:
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    DFRobot_EC10 ec;
    DFRobot_PH ph;

bool calibrationIsRunning = false;

void ads1115_set();
void ads1115();

void Analog_Input_Driver();
float getWaterTemperature()
{
    WT_Res = (float)(volts[0] * 1000) / (5 - volts[0]) * (AnInWT_Res / 1000); //R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
    WT = (float)(sqrt((-0.00232 * WT_Res) + 17.59246) - 3.908) / (-0.00116)  ; //Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116

	// sensors.requestTemperatures(); // Send the command to get temperatures
	// float WT = sensors.getTempCByIndex(0);

	if (WT == 85.00 || WT == -127.00) //take the last correct temperature value if getting 85 or -127 value
	{
		WT = lastTemperature;
	}
	else
	{
		lastTemperature = WT;
	}

    ESP_LOGD(analog,"WaterTemperature = %d", WT);
	return WT;
}

int i = 0;
bool readSerial(char result[])
{
	while (Serial.available() > 0)
	{
		char inChar = Serial.read();
		if (inChar == '\n')
		{
			result[i] = '\0';
			Serial.flush();
			i = 0;
			return true;
		}
		if (inChar != '\r')
		{
			result[i] = inChar;
			i++;
		}
		delay(1);
	}
	return false;
}
void ec_ph();
void ec_ph2();

void analog_sens()
{
    // if (this->AnInWT_Val_ != nullptr) { this->AnInWT_Val_->publish_state(WT); }
    // if (this->AnInVPow_Val_ != nullptr) { this->AnInVPow_Val_->publish_state(VPow); }
    // if (this->AnInLvl_Perc_ != nullptr) 
    // { 
    //     // for (size_t i = 0; i < sizeof(AnInLvl_Perc_); i++) 
    //     // this->AnInLvl_Perc_->publish_state(LvlPerc[i]);
    //     this->AnInLvl_Perc_->publish_state(LvlPerc);
    // }
    // if (this->AnInEC_Val_ != nullptr) 
    // {
    //     this->AnInEC_Val_->publish_state(ecValue);
    // }
    // if (this->AnInPH_Val_ != nullptr) 
    // {
    //     this->AnInPh_Val_->publish_state(phValue);
    // }
    // if (this->AnInGen_Val_ != nullptr) 
    // {
    //     this->AnInGen_Val_->publish_state(AnGen);
    // } 
}

protected:

};

}  // namespace water_quality
}  // namespace esphome