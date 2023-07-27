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

class Custom : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;

  void set_variables(double a);

protected:

};

template<typename... Ts> class Custom_action : public Action<Ts...> {
 public:
  Custom_action(Custom *cus) : custom_(cus) {}

  void play(Ts... x) override {
    this->custom_->set_variables(this->a_.value(x...));
  }
  TEMPLATABLE_VALUE(double, a)

 protected:
  Custom *custom_;
};

} //namespace custom
} //namespace esphome
