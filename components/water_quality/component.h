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

uint8_t data1[2];
// float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void setup() override;
void loop() override;

// void set_custom_data(uint8_t data[]) 
// { 
//     // data = &dat; 
//     // memcpy(&data1, &data, sizeof data1);
//     // data1 = data;
//     data1[0]=data[0];
//     data1[1]=data[1];
// }
void calibration(const std::vector<uint8_t> &cx) 
{ 
    Pump_Calib_X = cx;
    // Pump_Calib_Y = cy;
}

private:

std::vector<uint8_t> Pump_Calib_X{nullptr};
std::vector<uint8_t> Pump_Calib_Y{nullptr};

};
 
}  // namespace water_quality
}  // namespace esphome
