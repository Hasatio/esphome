// #include "water_quality.h"
#include "wq_pump.h"

namespace esphome {
namespace water_quality {

void Pump::pump_total()
{
    for (size_t i = 0; i < Pump_Type.size(); i++)
    {
        if (Pump_Type[i] == 1)
        {
            if (Pump_Dose[i] >= 1)
            {
                Pump_Total[i][1] += Pump_Dose[i]%2;
                if (Pump_Dose[i] == 1)
                Pump_Dose[i] = 0;
                else
                {
                    if (Pump_Mode[i] == 2)
                    {
                        Pump_Status[i] = 3;
                        break;
                    }
                    Pump_Dose[i] /= 2;
                    Pump_Total[i][0] += (int)(Pump_Total[i][0] + Pump_Dose[i]) / 1000;
                    Pump_Total[i][1] = (int)(Pump_Total[i][1] + Pump_Dose[i]) % 1000;
                }
            }
            Pump_Mode[i] = 0;
            Pump_Status[i] = 2;
        }
        if (Pump_Type[i] == 2)
        {
            if (Pump_Circulation[i] >= 1)
            {
                ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, Pump_Circulation[i]);
                Pump_Total[i][1] += Pump_Circulation[i]%2;
                if (Pump_Circulation[i] == 1)
                Pump_Circulation[i] = 0;
                else
                {
                    if (Pump_Mode[i] == 2)
                    {
                        Pump_Status[i] = 3;
                        Pump_Circulation[i] = 0;
                    }
                    Pump_Circulation[i] /= 2;
                    Pump_Total[i][0] += (int)(Pump_Total[i][0] + Pump_Circulation[i]) / 1000;
                    Pump_Total[i][1] = (int)(Pump_Total[i][1] + Pump_Circulation[i]) % 1000;
                }
            }
            Pump_Mode[i] = 0;
            Pump_Status[i] = 2;
        }
        ESP_LOGD(TAG,"Pump_Total[%d] = %d.%03d", i, Pump_Total[i][0], Pump_Total[i][1]);
    
    }
}

}  // namespace water_quality
}  // namespace esphome