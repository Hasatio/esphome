#pragma once

#ifndef ANALOG_H
#define ANALOG_H

#include "mux.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_EC.h"
#include "DFRobot_EC10.h"
#include "DFRobot_PH.h"

namespace esphome {
namespace water_quality {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Analog
{
public:

unsigned long intervals[10] = {
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

void ads1115_set();
void ads1115();

void Analog_Input_Driver();
float getWaterTemperature();

bool readSerial(char result[]);

void ec_ph();
void ec_ph2();

// extern Analog ana;

uint16_t adc[8], AnInWT_Res = 1000; //temperature sensor model pt1000 and its resistance is 1k
float volts[8]={0}, WT_Res, WT, VPow, LvlPerc[2], AnGen[2];
float  voltagePH, voltageEC, PH, EC, lastTemperature;
char cmd[10];
uint8_t tot, AnInGen_Ch[2], rnd;

float ecVoltage,phVoltage,temperature;

std::vector<uint16_t> AnInLvl_ResMin{0,0};
std::vector<uint16_t> AnInLvl_ResMax{0,0};
uint8_t AnInEC_Ch, AnInEC_Type, AnInPH_Ch, AnInPH_Type;

protected:

};

}  // namespace water_quality
}  // namespace esphome

#endif  // ANALOG_H