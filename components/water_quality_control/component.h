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

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void setup() override;
void loop() override;
void dump_config() override;

void pump(String PT[6],uint8_t PCX[8],uint8_t PCY[8],uint8_t PM[4],uint8_t PD[4]);

};


}  // namespace water_quality_control
}  // namespace esphome