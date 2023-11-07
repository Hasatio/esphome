#pragma once

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

static const char *const pump = "pump";

uint8_t dose, circ;
uint16_t PwmFreq = 1000;
std::vector<uint8_t> Pump_Mode{0,0,0,0,0,0};
std::vector<bool> Pump_Reset{0,0,0,0,0,0};
std::vector<uint16_t> Pump_Dose{0,0,0,0,0,0};
std::vector<uint16_t> Pump_Circulation{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Status{0,0,0,0,0,0};
std::vector<std::vector<uint16_t>> Pump_Total{{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};

class Pump
{
public:
void pump_total();

protected:

};

}  // namespace water_quality
}  // namespace esphome