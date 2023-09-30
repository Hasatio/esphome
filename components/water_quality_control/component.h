#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23008.h>
#include <Adafruit_PWMServoDriver.h>


namespace esphome {
namespace water_quality_control {

class Component : public Component {
 public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; } // çalışma önceliği

void setup() override;
void loop() override;
void dump_config() override;

};


}  // namespace water_quality_control
}  // namespace esphome