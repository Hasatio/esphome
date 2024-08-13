#include "mcs_digital.h"

namespace esphome {
namespace mcs {


void MCS_Digital::Digital_Output_Driver(bool output[])
{
    bool* out = get_Digital_Output();

    for (uint8_t i = 0; i < 20; i++)
        output[i] = out[i];
}

}  // namespace mcs
}  // namespace esphome