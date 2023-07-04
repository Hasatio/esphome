#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace custom {

class Custom : public output::FloatOutput, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void write_state(float state) override;
  void dump_config() override;
};


} //namespace custom
} //namespace esphome