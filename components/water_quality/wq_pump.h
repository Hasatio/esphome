#pragma once

#ifndef PUMP_H
#define PUMP_H

#include "esphome.h"
#include "esphome/core/log.h"
#include <cmath>
#include <thread>

namespace esphome {
namespace water_quality {

class Pump
{
public:
void Pump_driver(float pwm[]);
void Dosing_Controller(float pump[]);
void Circulation_Controller(float pump[]);

void set_Pump_Calib_Gain(uint8_t pcg[])     {for (size_t i = 0; i < 6; i++) Pump_Calib_Gain[i] = pcg[i];}
void set_Pump_Type(uint8_t pt[])            {for (size_t i = 0; i < 6; i++) Pump_Type[i] = pt[i];}
void set_Pump_Mode(uint8_t pm[])            {for (size_t i = 0; i < 6; i++) Pump_Mode[i] = pm[i];}
void set_Pump_Status(uint8_t ps[])          {for (size_t i = 0; i < 6; i++) Pump_Status[i] = ps[i];}
void set_Pump_Dose(uint16_t pd[])           {for (size_t i = 0; i < 6; i++) Pump_Dose[i] = pd[i];}
void set_Pump_Circulation(uint16_t pc[])    {for (size_t i = 0; i < 6; i++) Pump_Circulation[i] = pc[i];}
void set_Pump_Total(uint16_t pt[][2])       {for (size_t i = 0; i < 6; i++) for (size_t j = 0; j < 2; j++) Pump_Total[i][j] = pt[i][j];}
void set_Pump_Reset(bool pr[])              {for (size_t i = 0; i < 6; i++) Pump_Reset[i] = pr[i];}

uint8_t* get_Pump_Calib_Gain()      {return Pump_Calib_Gain;}
uint8_t* get_Pump_Type()            {return Pump_Type;}
uint8_t* get_Pump_Mode()            {return Pump_Mode;}
uint8_t* get_Pump_Status()          {return Pump_Status;}
uint16_t* get_Pump_Dose()           {return Pump_Dose;}
uint16_t* get_Pump_Circulation()    {return Pump_Circulation;}
uint16_t (*get_Pump_Total())[6][2]  {return &Pump_Total;}
bool* get_Pump_Reset()              {return Pump_Reset;}

protected:
uint8_t Pump_Calib_Gain[6] = {0};
uint8_t Pump_Type[6] = {0};
uint8_t Pump_Mode[6] = {0};
uint8_t Pump_Status[6] = {0};
uint16_t Pump_Dose[6] = {0};
uint16_t Pump_Circulation[6] = {0};
uint16_t Pump_Total[6][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
bool Pump_Reset[6] = {0};
};

}  // namespace water_quality
}  // namespace esphome

#endif  // PUMP_H