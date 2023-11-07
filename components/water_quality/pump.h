#pragma once

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

static const char *const TAG = "pump";

class Pump
{
public:

std::vector<float> Pump_Calib_Gain{0,0,0,0,0,0};
uint8_t dose, circ;
std::vector<uint8_t> Pump_Type{0,0,0,0,0,0};
std::vector<uint16_t> Pump_Dose{0,0,0,0,0,0};
std::vector<uint16_t> Pump_Circulation{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Mode{0,0,0,0,0,0};
std::vector<bool> Pump_Reset{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Status{0,0,0,0,0,0};
std::vector<std::vector<uint16_t>> Pump_Total{{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};

void pump_total();
void sensor();
};

}  // namespace water_quality
}  // namespace esphome