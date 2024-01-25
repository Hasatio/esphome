#include "wq_digital.h"

namespace esphome {
namespace water_quality {

void Digital::Digital_Input_Driver(bool inputs[])
{
    bool* in = get_Digital_In();

    for (size_t i = 0; i < 4; i++)
    {
        if (inputs[i])
            this->DigIn_FilterCoeff[i]++;
        else
            this->DigIn_FilterCoeff[i]--;

        if (this->DigIn_FilterCoeff[i] >= 8)
        {
            in[i] = 1;
            this->DigIn_FilterCoeff[i] = 4;
        }
        else if (this->DigIn_FilterCoeff[i] == 0)
        {
            in[i] = 0;
            this->DigIn_FilterCoeff[i] = 4;
        }
    }
}
void Digital::Digital_Output_Driver(bool outputs[])
{
    bool* out = get_Digital_Out();

    for (size_t i = 0; i < 4; i++)
    {
        outputs[i] = out[i];
    }
}

}  // namespace water_quality
}  // namespace esphome