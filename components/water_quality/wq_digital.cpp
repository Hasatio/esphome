#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_digital.h"

namespace esphome {
namespace water_quality {

void Digital::Digital_Input_Driver(bool inputs[])
{
    bool* digital = get_Digital_In();

    for (size_t i = 0; i < 8; i++)
    {
        if (inputs[i])
            DigIn_FilterCoeff[i]++;
        else
            DigIn_FilterCoeff[i]--;

        if (DigIn_FilterCoeff[i] > 8)
            DigIn_FilterCoeff[i] = 8;

        if (DigIn_FilterCoeff[i] == 0)
        {
            digital[i] = 0;
            DigIn_FilterCoeff[i] = 4;
        }
        else if (DigIn_FilterCoeff[i] == 8)
        {
            digital[i] = 1;
            DigIn_FilterCoeff[i] = 4;
        }
    }
}
void Digital::Digital_Output_Driver(bool outputs[])
{
    bool* digital = get_Digital_Out();

    for (size_t i = 0; i < 4; i++)
        outputs[i] = digital[i];
}

}  // namespace water_quality
}  // namespace esphome