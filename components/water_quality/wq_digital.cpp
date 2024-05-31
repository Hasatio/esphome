#include "wq_digital.h"

namespace esphome {
namespace water_quality {

void Digital_Input_Filter(bool input[], Digital* digital)
{
    uint8_t filteramount = 10; // Total amount of input to filter
    uint8_t timeperiod = 10; // Wait time before each update
    uint8_t* filter = digital->get_Digital_FilterCoeff();

    for (size_t i = 0; i < 4; i++)
    {
        if (millis() - digital->get_Digital_Timepoint() >= timeperiod)
        {
            if (input[i])
                filter[i]++;
            else
                filter[i]--;

            digital->set_Digital_Timepoint(millis());
        }
        
        if (filter[i] >= filteramount)
        {
            filter[i] = filteramount / 2;
            input[i] = 1;
        }
        else if (filter[i] == 0)
        {
            filter[i] = filteramount / 2;
            input[i] = 0;
        }
    }
}
void Digital::Digital_Input_Driver(bool input[])
{
    bool* in = get_Digital_Input();
    Digital_Input_Filter(input, this);

    for (size_t i = 0; i < 4; i++)
        in[i] = input[i];
}
void Digital::Digital_Output_Driver(bool output[])
{
    bool* out = get_Digital_Output();

    for (size_t i = 0; i < 4; i++)
        output[i] = out[i];
}

}  // namespace water_quality
}  // namespace esphome