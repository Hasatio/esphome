#pragma once

#ifndef WQ_PUMP_H
#define WQ_PUMP_H

#include "esphome.h"
#include "esphome/core/log.h"
#include <cmath>
#include <thread>
#include <esp_timer.h>

namespace esphome {
namespace water_quality {

class Pump
{
public:
void Timer_Setup(float period);
static void IRAM_ATTR Timer(void* arg);
void Calibration_Controller();

void Generic_Pump_Driver(float pwm[]);
void Dosing_Controller(float pump[]);
void Circulation_Controller(float pump[]);

void set_Pump_Calibration_Mode_Check(bool check)            { Pump_Calibration_Mode_Check = check; }
void set_Pump_Calibration_Condition(uint8_t condition[])    { for (uint8_t i = 0; i < 6; i++)    Pump_Calibration_Condition[i] = condition[i]; }
void set_Pump_Calibration_Mode(bool mode[])                 { for (uint8_t i = 0; i < 6; i++)    Pump_Calibration_Mode[i] = mode[i]; }
void set_Pump_Calibration_Volume(uint8_t volume[])          { for (uint8_t i = 0; i < 6; i++)    Pump_Calibration_Volume[i] = volume[i]; }
void set_Pump_Calibration_Gain(float gain[])                { for (uint8_t i = 0; i < 6; i++)    Pump_Calibration_Gain[i] = gain[i]; }
void set_Pump_Type(uint8_t type[])                          { for (uint8_t i = 0; i < 6; i++)    Pump_Type[i] = type[i]; }
void set_Pump_Mode(uint8_t mode[])                          { for (uint8_t i = 0; i < 6; i++)    Pump_Mode[i] = mode[i]; }
void set_Pump_Status(uint8_t status[])                      { for (uint8_t i = 0; i < 6; i++)    Pump_Status[i] = status[i]; }
void set_Pump_Dose(float dose[])                            { for (uint8_t i = 0; i < 6; i++)    Pump_Dose[i] = dose[i]; }
void set_Pump_Circulation(float circulation[])              { for (uint8_t i = 0; i < 6; i++)    Pump_Circulation[i] = circulation[i]; }
void set_Pump_Total(uint32_t total[][2])                    { for (uint8_t i = 0; i < 6; i++)    for (uint8_t j = 0; j < 2; j++)  Pump_Total[i][j] = total[i][j]; }
void set_Pump_Reset(bool reset[])                           { for (uint8_t i = 0; i < 6; i++)    Pump_Reset[i] = reset[i]; }
void set_Pump_Time(float time[])                            { for (uint8_t i = 0; i < 6; i++)    Pump_Time[i] = time[i]; }
void set_Min_Time(float min)                                { Min_Time = min; }

bool get_Pump_Calibration_Mode_Check()      { return Pump_Calibration_Mode_Check; }
uint8_t* get_Pump_Calibration_Condition()   { return Pump_Calibration_Condition; }
bool* get_Pump_Calibration_Mode()           { return Pump_Calibration_Mode; }
uint8_t* get_Pump_Calibration_Volume()      { return Pump_Calibration_Volume; }
float* get_Pump_Calibration_Gain()          { return Pump_Calibration_Gain; }
uint8_t* get_Pump_Type()                    { return Pump_Type; }
uint8_t* get_Pump_Mode()                    { return Pump_Mode; }
uint8_t* get_Pump_Status()                  { return Pump_Status; }
float* get_Pump_Dose()                      { return Pump_Dose; }
float* get_Pump_Circulation()               { return Pump_Circulation; }
uint32_t (*get_Pump_Total())[2]             { return Pump_Total; }
bool* get_Pump_Reset()                      { return Pump_Reset; }
float* get_Pump_Time()                      { return Pump_Time; }
float get_Min_Time()                        { return Min_Time; }
uint8_t get_calib_time()                    { return calib_time; }

protected:
bool Pump_Calibration_Mode_Check = 0;
uint8_t Pump_Calibration_Condition[6] = {0};
bool Pump_Calibration_Mode[6] = {0};
uint8_t Pump_Calibration_Volume[6] = {0};
float Pump_Calibration_Gain[6] = {0};
uint8_t Pump_Type[6] = {0};
uint8_t Pump_Mode[6] = {0};
uint8_t Pump_Status[6] = {0};
float Pump_Dose[6] = {0};
float Pump_Circulation[6] = {0};
uint32_t Pump_Total[6][2] = {0};
bool Pump_Reset[6] = {0};
float Pump_Time[6] = {0};
float Min_Time = 0;
uint8_t calib_time = 120;

private:
esp_timer_handle_t timer;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WQ_PUMP_H