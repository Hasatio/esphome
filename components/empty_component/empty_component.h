#pragma once

#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"

namespace esphome {
namespace empty_component {

class EmptyComponent : public output::FloatOutput, public Component {
 public:
  void setup() override;
  void loop() override;
  void write_state(float state) override;
  void dump_config() override;
};

}  // namespace empty_component
}  // namespace esphome
