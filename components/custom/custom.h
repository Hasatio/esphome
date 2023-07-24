#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/log.h"

//#include "esphome\components\uart\uart.h"
//#include "esphome\components\uart\uart_component.h"

namespace esphome {
namespace custom {

class Custom : public output::FloatOutput, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void write_state(float state) override;
  void loop() override;
  void dump_config() override;


  void set_baud_rate(uint32_t baud_rate) { baud_rate_ = baud_rate; }
  uint32_t get_baud_rate() const { return baud_rate_; }

  void check_uart_settings(uint32_t baud_rate);

protected:
  uint32_t baud_rate_;

};


} //namespace custom
} //namespace esphome
