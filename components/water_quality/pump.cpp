#include "water_quality.h"
#include "pump.h"

namespace esphome {
namespace water_quality {

void Pump::pump_total()
{
    for (size_t i = 0; i < pum.Pump_Type.size(); i++)
    {
        if (pum.Pump_Type[i] == 1)
        {
            if (pum.Pump_Dose[i] >= 1)
            {
                pum.Pump_Total[i][1] += pum.Pump_Dose[i]%2;
                if (pum.Pump_Dose[i] == 1)
                pum.Pump_Dose[i] = 0;
                else
                {
                    if (pum.Pump_Mode[i] == 2)
                    {
                        pum.Pump_Status[i] = 3;
                        break;
                    }
                    pum.Pump_Dose[i] /= 2;
                    pum.Pump_Total[i][0] += (int)(pum.Pump_Total[i][0] + pum.Pump_Dose[i]) / 1000;
                    pum.Pump_Total[i][1] = (int)(pum.Pump_Total[i][1] + pum.Pump_Dose[i]) % 1000;
                }
            }
            pum.Pump_Mode[i] = 0;
            pum.Pump_Status[i] = 2;
        }
        if (pum.Pump_Type[i] == 2)
        {
            if (pum.Pump_Circulation[i] >= 1)
            {
                ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pum.Pump_Circulation[i]);
                pum.Pump_Total[i][1] += pum.Pump_Circulation[i]%2;
                if (pum.Pump_Circulation[i] == 1)
                pum.Pump_Circulation[i] = 0;
                else
                {
                    if (pum.Pump_Mode[i] == 2)
                    {
                        pum.Pump_Status[i] = 3;
                        pum.Pump_Circulation[i] = 0;
                    }
                    pum.Pump_Circulation[i] /= 2;
                    pum.Pump_Total[i][0] += (int)(pum.Pump_Total[i][0] + pum.Pump_Circulation[i]) / 1000;
                    pum.Pump_Total[i][1] = (int)(pum.Pump_Total[i][1] + pum.Pump_Circulation[i]) % 1000;
                }
            }
            pum.Pump_Mode[i] = 0;
            pum.Pump_Status[i] = 2;
        }
        ESP_LOGD(TAG,"Pump_Total[%d] = %d.%03d", i, pum.Pump_Total[i][0], pum.Pump_Total[i][1]);
    
    }
}

}  // namespace water_quality
}  // namespace esphome