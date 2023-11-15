#pragma once

// #ifndef ANALOG_H
// #define ANALOG_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include "DFRobot_EC.h"
#include "DFRobot_EC10.h"
#include "DFRobot_PH.h"

namespace esphome {
namespace water_quality {

void Analog_Input_Driver(float volts[]);

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

void set_WT_Res(uint16_t set)		{WT_Res = set;}
void set_WT(float set)				{WT = set;}
void set_VPow(float set)			{VPow = set;}
void set_AnInLvl_Perc(float set[])	{for (size_t i = 0; i < 2; i++)	AnInLvl_Perc[i] = set[i];}
void set_AnInPH_Val(float set)		{AnInPH_Val = set;}
void set_AnInEC_Val(float set)		{AnInEC_Val = set;}
void set_AnInGen_Val(float set[])	{for (size_t i = 0; i < 2; i++)	AnInGen_Val[i] = set[i];}

uint16_t get_WT_Res()		{return WT_Res;}
float get_WT()				{return WT;}
float get_VPow()			{return VPow;}
float get_AnInLvl_Perc()	{return AnInLvl_Perc;}
float get_AnInPH_Val()		{return AnInPH_Val;}
float get_AnInEC_Val()		{return AnInEC_Val;}
float get_AnInGen_Val()		{return AnInGen_Val;}

bool readSerial(char result[]);

void ec_ph();
void ec_ph2();

// extern Analog ana;

uint16_t WT_Res = 1000; //temperature sensor model pt1000 and its resistance is 1k
float AnInWT_Val, AnInVPow_Val, AnInLvl_Perc[2], AnInPH_Val, AnInEC_Val, AnInGen_Val[2];
std::vector<uint16_t> AnInLvl_ResMin{0,0};
std::vector<uint16_t> AnInLvl_ResMax{0,0};
uint8_t AnInEC_Ch, AnInEC_Type, AnInPH_Ch, AnInPH_Type, AnInGen_Ch[2];
float voltagePH, voltageEC, lastTemperature;
char cmd[10];

float ecVoltage,phVoltage,temperature;

protected:

};

}  // namespace water_quality
}  // namespace esphome

// #endif  // ANALOG_H