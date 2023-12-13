#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_digital.h"

namespace esphome {
namespace water_quality {

void Digital::Digital_Input_Driver(bool inputs[])
{
  set_Digital_In(inputs);
}
void Digital::Digital_Output_Driver(bool outputs[])
{
  set_Digital_Out(outputs);
}
}  // namespace water_quality
}  // namespace esphome