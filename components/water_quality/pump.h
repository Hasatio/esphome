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
void setdose(uint8_t set)   {dose = set;}
uint8_t getdose()   {return dose;}
void ddose()    {ESP_LOGI("dose","Pump_dose = %d", dose);}

void pump_total();
void sensor();

uint8_t dose, circ;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // PUMP_H