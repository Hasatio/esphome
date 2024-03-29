#pragma once

#ifndef PUMP_H
#define PUMP_H

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

class Pump
{
public:
<<<<<<< HEAD
void Pump_driver(float pwm[]);
void Dosing_Controller(float pump[]);
void Circulation_Controller(float pump[]);
=======
void set_Pump_Dose(uint16_t pd[])  {for (size_t i = 0; i < 6; i++) Pump_Dose[i] = pd[i];}
void set_Pump_Status(uint8_t ps[])  {for (size_t i = 0; i < 6; i++) Pump_Status[i] = ps[i];}
>>>>>>> parent of 3b0422f (pump and whole installation)

uint16_t* get_Pump_Dose() {return Pump_Dose;}
uint8_t* get_Pump_Status() {return Pump_Status;}

<<<<<<< HEAD
protected:
uint8_t Pump_Calib_Gain[6] = {0};
uint8_t Pump_Type[6] = {0};
uint8_t Pump_Mode[6] = {0};
uint8_t Pump_Status[6] = {0};
uint16_t Pump_Dose[6] = {0};
uint16_t Pump_Circulation[6] = {0};
uint16_t Pump_Total[6][2] = {0};
bool Pump_Reset[6] = {0};
=======
void ddose()    {ESP_LOGI("dose","Pump_dose = %d", dose);}

void pump_total();
void sensor();

uint8_t dose, circ;
std::vector<float> Pump_Calib_Gain{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Type{0,0,0,0,0,0};
uint16_t Pump_Dose[6] = {0};
std::vector<uint16_t> Pump_Circulation{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Mode{0,0,0,0,0,0};
std::vector<bool> Pump_Reset{0,0,0,0,0,0};
uint8_t Pump_Status[6] = {0};
std::vector<std::vector<uint16_t>> Pump_Total{{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
>>>>>>> parent of 3b0422f (pump and whole installation)
};

}  // namespace water_quality
}  // namespace esphome

#endif  // PUMP_H