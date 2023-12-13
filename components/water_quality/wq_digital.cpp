#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_digital.h"

namespace esphome {
namespace water_quality {

void Digital::Digital_Input_Driver(bool inputs[])
{
  bool digital[4];

  for (size_t i = 0; i < 4; i++)
    digital[i] = inputs[i];

  set_Digital_In(digital);
}
void Digital::Digital_Output_Driver(bool outputs[])
{
  bool* digital = get_Digital_Out();
  
  for (size_t i = 0; i < 4; i++)
    outputs[i] = digital[i];
}
}  // namespace water_quality
}  // namespace esphome