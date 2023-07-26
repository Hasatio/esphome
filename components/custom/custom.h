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
  // uint32_t get_baud_rate() const { return baud_rate_; }

  void check_uart_settings(uint32_t baud_rate);
  void set_variables(double *var);

protected:
  uint32_t baud_rate_;

};

template<typename... Ts> class Custom_action : public Action<Ts...> {
 public:
  Custom_action(Custom *cus) : custom_(cus) {}

  void play(Ts... x) override {
    this->custom_->set_variables(this->on_custom_.value(x...));
  }
  TEMPLATABLE_VALUE(double, var)

 protected:
  Custom *custom_;
};

} //namespace custom
} //namespace esphome
