#pragma once

#ifndef WQ_ANALOG_H
#define WQ_ANALOG_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "DFRobot_EC.h"
#include "DFRobot_EC10.h"
#include "DFRobot_PH.h"
#include <cmath>

namespace esphome {
namespace water_quality {

class Analog
{
public:

void Analog_Input_Driver(float volts[]);

void set_version(uint8_t ver)                   { version = ver; }
void set_WatTemp_Res(uint16_t res)              { AnInWatTemp_Res = res; }
void set_WatTemp_Val(float wt)                  { AnInWatTemp_Val = wt; }
void set_VoltPow_Val(float vp)                  { AnInVoltPow_Val = vp; }
void set_ResMax(uint16_t resmax[])              { for (uint8_t i = 0; i < 2; i++)    AnInLvl_ResMax[i] = resmax[i]; }
void set_ResMin(uint16_t resmin[])              { for (uint8_t i = 0; i < 2; i++)    AnInLvl_ResMin[i] = resmin[i]; }
void set_Lvl_Perc(float lvl[])                  { for (uint8_t i = 0; i < 2; i++)    AnInLvl_Perc[i] = lvl[i]; }
void set_PH_Ch(uint8_t ph)                      { AnInPH_Ch = ph; }
void set_PH_Type(uint8_t ph)                    { AnInPH_Type = ph; }
void set_PH_Cal(float ph[][2])                  { for (uint8_t i = 0; i < 2; i++)    for (uint8_t j = 0; j < 2; j++)  AnInPH_Cal[i][j] = ph[i][j]; }
void set_PH_Val(float ph)                       { AnInPH_Val = ph; }
void set_PH_Cal_Volt(float ph)                  { phCalVoltage = ph; }
void set_EC_Ch(uint8_t ec)                      { AnInEC_Ch = ec; }
void set_EC_Type(uint8_t ec)                    { AnInEC_Type = ec; }
void set_EC_Cal(float ec[][2])                  { for (uint8_t i = 0; i < 2; i++)    for (uint8_t j = 0; j < 2; j++)  AnInEC_Cal[i][j] = ec[i][j]; }
void set_EC_Val(float ec)                       { AnInEC_Val = ec; }
void set_EC_Cal_Volt(float ec)                  { ecCalVoltage = ec; }
void set_Gen_Ch(float gen[])                    { for (uint8_t i = 0; i < 2; i++)    AnInGen_Ch[i] = gen[i]; }
void set_Gen_Val(float gen[])                   { for (uint8_t i = 0; i < 2; i++)    AnInGen_Val[i] = gen[i]; }

uint8_t get_version()                   { return version; }	
uint16_t get_WatTemp_Res()              { return AnInWatTemp_Res; }
float get_WatTemp_Val()                 { return AnInWatTemp_Val; }
float get_VoltPow_Val()                 { return AnInVoltPow_Val; }
uint16_t* get_ResMin()                  { return AnInLvl_ResMin; }
uint16_t* get_ResMax()                  { return AnInLvl_ResMax; }
float* get_Lvl_Perc()                   { return AnInLvl_Perc; }
uint8_t get_PH_Ch()                     { return AnInPH_Ch; }
uint8_t get_PH_Type()                   { return AnInPH_Type; }
float (*get_PH_Cal())[2]                { return AnInPH_Cal; }
float get_PH_Val()                      { return AnInPH_Val; }
float get_PH_Cal_Volt()                 { return phCalVoltage; }
uint8_t get_EC_Ch()                     { return AnInEC_Ch; }
uint8_t get_EC_Type()                   { return AnInEC_Type; }
float (*get_EC_Cal())[2]                { return AnInEC_Cal; }
float get_EC_Val()                      { return AnInEC_Val; }
float get_EC_Cal_Volt()                 { return ecCalVoltage; }
float* get_Gen_Ch()                     { return AnInGen_Ch; }
float* get_Gen_Val()                    { return AnInGen_Val; }

protected:
uint8_t version;
uint16_t AnInWatTemp_Res = 1000; // Temperature sensor model pt1000 and its resistance is 1k
float AnInWatTemp_Val = 0;
float AnInVoltPow_Val = 0;
uint16_t AnInLvl_ResMin[2] = {0}, AnInLvl_ResMax[2] = {0};
float AnInLvl_Perc[2] = {0};
uint8_t AnInPH_Ch = 0, AnInPH_Type = 0;
float AnInPH_Cal[2][2] = {0};
float AnInPH_Val = 0;
float phCalVoltage = 0;
uint8_t AnInEC_Ch = 0, AnInEC_Type = 0;
float AnInEC_Cal[2][2] = {0};
float AnInEC_Val = 0;
float ecCalVoltage = 0;
float AnInGen_Ch[2] = {0};
float AnInGen_Val[2] = {0};
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WQ_ANALOG_H