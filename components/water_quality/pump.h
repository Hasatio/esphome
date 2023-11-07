#pragma once

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

std::vector<float> Pump_Calib_Gain{};
std::vector<uint8_t> Pump_Type{};
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
void pump_calib_gain(const std::vector<float> &pcg);
void pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c);
void pump_dose(std::vector<uint16_t> &pdose);
void pump_circulation(std::vector<uint16_t> &pcirc);
void pump_total();
void pump_mode(std::vector<uint8_t> &pmode);
void pump_reset(std::vector<bool> &pres);

protected:

};

}  // namespace water_quality
}  // namespace esphome