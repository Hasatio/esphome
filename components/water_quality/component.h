#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include "Adafruit_MCP23X08.h"
#include <Adafruit_PWMServoDriver.h>

#include <vector>

namespace esphome {
namespace water_quality {

class MyComponent : public Component 
{
public:

// float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void setup() override;
void loop() override;

void set_custom_data(std::vector<uint8_t> &data) { custom_data = data; }

std::vector<uint8_t> custom_data{};

};
 
}  // namespace water_quality
}  // namespace esphome
