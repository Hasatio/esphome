#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace water_quality {

class MyComponent : public Component 
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void setup() override;
void loop() override;

void calibration(const std::vector<uint8_t> &cx1, const std::vector<uint8_t> &cy1, const std::vector<uint8_t> &cx2, const std::vector<uint8_t> &cy2, const std::vector<uint8_t> &cx3, const std::vector<uint8_t> &cy3, const std::vector<uint8_t> &cx4, const std::vector<uint8_t> &cy4) 
{ 
    this->Pump_Calib_X1 = cx1;
    this->Pump_Calib_Y1 = cy1;
    this->Pump_Calib_X2 = cx2;
    this->Pump_Calib_Y2 = cy2;
    this->Pump_Calib_X3 = cx3;
    this->Pump_Calib_Y3 = cy3;
    this->Pump_Calib_X4 = cx4;
    this->Pump_Calib_Y4 = cy4;
}

private:

std::vector<uint8_t> Pump_Calib_X1{};
std::vector<uint8_t> Pump_Calib_Y1{};
std::vector<uint8_t> Pump_Calib_X2{};
std::vector<uint8_t> Pump_Calib_Y2{};
std::vector<uint8_t> Pump_Calib_X3{};
std::vector<uint8_t> Pump_Calib_Y3{};
std::vector<uint8_t> Pump_Calib_X4{};
std::vector<uint8_t> Pump_Calib_Y4{};

};
 
}  // namespace water_quality
}  // namespace esphome
