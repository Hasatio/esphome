#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_EC10.h"
#include "DFRobot_PH.h"

namespace esphome {
namespace water_quality {

#define ADS1X15_ADDRESS1 0x48
#define ADS1X15_ADDRESS2 0x49

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
    DFRobot_EC10 ec;
    DFRobot_PH ph;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;
    
bool calibrationIsRunning = false;

void ads1115_set()
{ 
    // tcaselect(0);
    if (!ads1.begin(ADS1X15_ADDRESS1))
    {
      ESP_LOGE(analog,"Failed to initialize ADS1115_1.");
    //   while (1);
    }
    if (!ads2.begin(ADS1X15_ADDRESS2))
    {
      ESP_LOGE(analog,"Failed to initialize ADS1115_2.");
    //   while (1);
    }

    // The ADC input range (or gain) can be changed via the following
    // functions, but be careful never to exceed VDD +0.3V max, or to
    // exceed the upper and lower limits if you adjust the input range!
    // Setting these values incorrectly may destroy your ADC!
    
    //                                          ADS1015          ADS1115
    //                                          -------          -------
    // GAIN_TWOTHIRDS  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // GAIN_ONE        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // GAIN_TWO        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // GAIN_FOUR       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // GAIN_EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // GAIN_SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    ads1.setGain(GAIN_TWOTHIRDS);
    ads2.setGain(GAIN_TWOTHIRDS);
    
    // RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
    // RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
    // RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
    // RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
    // RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
    // RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
    // RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
    // RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second
    ads1.setDataRate(RATE_ADS1115_860SPS);
    ads2.setDataRate(RATE_ADS1115_860SPS);
    
    // ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
    // ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000) ///< Differential P = AIN0, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000) ///< Differential P = AIN1, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000) ///< Differential P = AIN2, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
    // ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
    // ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
    // ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3
    // ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1);

    ec.begin();
    ph.begin();
}
void ads1115()
{
    // tcaselect(0);
    for(size_t i = 0; i < 4; i++)
    {
        adc[i] = ads1.readADC_SingleEnded(i%4);
        volts[i] = ads1.computeVolts(adc[i]);
        // ESP_LOGD(analog,"ads%d = %f", i+1, volts[i]);
    }
    for(size_t i = 4; i < 8; i++){
        adc[i] = ads2.readADC_SingleEnded(i%4);
        volts[i] = ads2.computeVolts(adc[i]);
        // ESP_LOGD(analog,"ads%d = %f", i+1, volts[i]);
    }
}

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