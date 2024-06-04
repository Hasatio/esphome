#pragma once

#ifndef ANALOG_H
#define ANALOG_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "DFRobot_EC.h"
#include "DFRobot_EC10.h"
#include "DFRobot_PH.h"
#include <EEPROM.h>

namespace esphome {
namespace water_quality {

class Analog
{
public:

void Analog_Input_Driver(float volts[]);

void set_Analog_Timepoint(unsigned long time)   { AnIn_Timepoint = time; }
void set_version(uint8_t ver)                   { version = ver; }
void set_WaterTemp_Res(uint16_t res)            { AnInWTemp_Res = res; }
void set_WaterTemp_Val(float wt)                { AnInWTemp_Val = wt; }
void set_VoltagePow_Val(float vp)               { AnInVPow_Val = vp; }
void set_ResMin(uint16_t resmax[])              { for (size_t i = 0; i < 2; i++)    AnInLvl_ResMin[i] = resmax[i]; }
void set_ResMax(uint16_t resmin[])              { for (size_t i = 0; i < 2; i++)    AnInLvl_ResMax[i] = resmin[i]; }
void set_Level_Perc(float lvl[])                { for (size_t i = 0; i < 2; i++)    AnInLvl_Perc[i] = lvl[i]; }
void set_PH_Ch(uint8_t ph)                      { AnInPH_Ch = ph; }
void set_PH_Type(uint8_t ph)                    { AnInPH_Type = ph; }
void set_PH_Calibration(bool cal)               { PH_Calibration = cal; }
void set_PH_Cal(float cal)                      { AnInPH_Cal = cal; }
void set_PH_Val(float ph)                       { AnInPH_Val = ph; }
void set_EC_Ch(uint8_t ec)                      { AnInEC_Ch = ec; }
void set_EC_Type(uint8_t ec)                    { AnInEC_Type = ec; }
void set_EC_Calibration(bool cal)               { EC_Calibration = cal; }
void set_EC_Cal(float cal)                      { AnInEC_Cal = cal; }
void set_EC_Val(float ec)                       { AnInEC_Val = ec; }
void set_Gen_Val(float gen[])                   { for (size_t i = 0; i < 2; i++)    AnInGen_Val[i] = gen[i]; }

unsigned long get_Analog_Timepoint()    { return AnIn_Timepoint; }
uint8_t get_version()                   { return version; }	
uint16_t get_WaterTemp_Res()            { return AnInWTemp_Res; }
float get_WaterTemp_Val()               { return AnInWTemp_Val; }
float get_VoltagePow_Val()              { return AnInVPow_Val; }
uint16_t* get_ResMin()                  { return AnInLvl_ResMin; }
uint16_t* get_ResMax()                  { return AnInLvl_ResMax; }
float* get_Level_Perc()                 { return AnInLvl_Perc; }
uint8_t get_PH_Ch()                     { return AnInPH_Ch; }
uint8_t get_PH_Type()                   { return AnInPH_Type; }
bool get_PH_Calibration()               { return PH_Calibration; }
float get_PH_Cal()                      { return AnInPH_Cal; }
float get_PH_Val()                      { return AnInPH_Val; }
uint8_t get_EC_Ch()                     { return AnInEC_Ch; }
uint8_t get_EC_Type()                   { return AnInEC_Type; }
bool get_EC_Calibration()               { return EC_Calibration; }
float get_EC_Cal()                      { return AnInEC_Cal; }
float get_EC_Val()                      { return AnInEC_Val; }
float* get_Gen_Val()                    { return AnInGen_Val; }

char cmd[10];

float ecVoltage,phVoltage,temperature;
unsigned long now = 0;

protected:
unsigned long AnIn_Timepoint = 0;
uint8_t version;
uint16_t AnInWTemp_Res = 1000; // Temperature sensor model pt1000 and its resistance is 1k
float AnInWTemp_Val = 0;
float AnInVPow_Val = 0;
uint16_t AnInLvl_ResMin[2] = {0}, AnInLvl_ResMax[2] = {0};
float AnInLvl_Perc[2] = {0};
uint8_t AnInPH_Ch = 0, AnInPH_Type = 0;
bool PH_Calibration = 0;
float AnInPH_Cal = 0;
float AnInPH_Val = 0;
uint8_t AnInEC_Ch = 0, AnInEC_Type = 0;
bool EC_Calibration = 0;
float AnInEC_Cal = 0;
float AnInEC_Val = 0;
float AnInGen_Val[2] = {0};
};

}  // namespace water_quality
}  // namespace esphome

#endif  // ANALOG_H