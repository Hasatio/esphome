#include "wq_digital.h"

namespace esphome {
namespace water_quality {

void Digital::Digital_Input_Driver(bool inputs[])
{
    bool* digital = get_Digital_In();

    for (size_t i = 0; i < 8; i++)
    {
        if (inputs[i])
            this->DigIn_FilterCoeff[i]++;
        else
            this->DigIn_FilterCoeff[i]--;

        if (this->DigIn_FilterCoeff[i] >= 8)
        {
            digital[i] = 1;
            this->DigIn_FilterCoeff[i] = 4;
        }
        else if (this->DigIn_FilterCoeff[i] == 0)
        {
            digital[i] = 0;
            this->DigIn_FilterCoeff[i] = 4;
        }
    }
    set_Digital_In(digital);
}
void Digital::Digital_Output_Driver(bool outputs[])
{
    bool* digital = get_Digital_Out();

    for (size_t i = 0; i < 4; i++)
        if (outputs[i])
            digital[i] = 1;
        else
            digital[i] = 0;
            
    set_Digital_Out(digital);
}

}  // namespace water_quality
}  // namespace esphome