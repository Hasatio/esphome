#pragma once

#ifndef ANALOG_H
#define ANALOG_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "DFRobot_EC.h"
#include "DFRobot_EC10.h"
#include "DFRobot_PH.h"

namespace esphome {
namespace water_quality {

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

void Analog_Input_Driver(float volts[]);

void set_WT_Res(uint16_t wtres)		{AnInWT_Res = wtres;}
void set_WT_Val(float wt)			{AnInWT_Val = wt;}
void set_VPow_Val(float vpow)		{AnInVPow_Val = vpow;}
void set_ResMin(uint16_t resmax[])	{for (size_t i = 0; i < 2; i++)	AnInLvl_ResMin[i] = resmax[i];}
void set_ResMax(uint16_t resmin[])	{for (size_t i = 0; i < 2; i++)	AnInLvl_ResMax[i] = resmin[i];}
void set_Lvl_Perc(float lvl[])		{for (size_t i = 0; i < 2; i++)	AnInLvl_Perc[i] = lvl[i];}
void set_EC_Val(float ec)			{AnInEC_Val = ec;}
void set_EC_Ch(uint8_t ec)			{AnInEC_Ch = ec;}
void set_EC_Type(uint8_t ec)		{AnInEC_Type = ec;}
void set_PH_Val(float ph)			{AnInPH_Val = ph;}
void set_PH_Ch(uint8_t ph)			{AnInPH_Ch = ph;}
void set_PH_Type(uint8_t ph)		{AnInPH_Type = ph;}
void set_Gen_Val(float gen[])		{for (size_t i = 0; i < 2; i++)	AnInGen_Val[i] = gen[i];}

uint16_t get_WT_Res() 	{return AnInWT_Res;}
float get_WT_Val()		{return AnInWT_Val;}
float get_VPow_Val()	{return AnInVPow_Val;}
uint16_t* get_ResMin()	{return AnInLvl_ResMin;}
uint16_t* get_ResMax()  {return AnInLvl_ResMax;}
float* get_Lvl_Perc()	{return AnInLvl_Perc;}
float get_EC_Val()		{return AnInEC_Val;}
uint8_t get_EC_Ch()		{return AnInEC_Ch;}
uint8_t get_EC_Type() 	{return AnInEC_Type;}
float get_PH_Val()		{return AnInPH_Val;}
uint8_t get_PH_Ch()		{return AnInPH_Ch;}
uint8_t get_PH_Type() 	{return AnInPH_Type;}
float* get_Gen_Val()	{return AnInGen_Val;}

bool readSerial(char result[]);

void ec_ph();
void ec_ph2();

uint8_t ver;
float voltagePH, voltageEC;
char cmd[10];

float ecVoltage,phVoltage,temperature;

protected:
float AnInWT_Val;
float AnInVPow_Val;
float AnInLvl_Perc[2];
float AnInEC_Val;
float AnInPH_Val;
float AnInGen_Val[2];
uint8_t AnInEC_Ch, AnInEC_Type;
uint8_t AnInPH_Ch, AnInPH_Type;
uint16_t AnInWT_Res = 1000; //temperature sensor model pt1000 and its resistance is 1k
uint16_t AnInLvl_ResMin[2], AnInLvl_ResMax[2];
};

}  // namespace water_quality
}  // namespace esphome

#endif  // ANALOG_H