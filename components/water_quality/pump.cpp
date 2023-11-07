#include "pump.h"
#include "water_quality.h"

namespace esphome {
namespace water_quality {

void MyComponent::pump_dose(std::vector<uint16_t> &pdose)
{
    // pdose.resize(dose);
bool ps[Pump_Type.size()] = {false};

    if (Pump_Dose != pdose)
    {
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            if (Pump_Status[i] != 1)
                ps[i] = true;
            if (Pump_Status[i] == 1 && ps)
            {
                Pump_Dose[i] = pdose[i];
                ps[i] = false;
            }
            else
                Pump_Dose[i] += pdose[i];
            ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, Pump_Dose[i]);
        }
    }
}
void MyComponent::pump_circulation(std::vector<uint16_t> &pcirc)
{
    // pcirc.resize(circ);

    if (Pump_Circulation != pcirc)
    {
        this->Pump_Circulation = pcirc;
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, Pump_Circulation[i]);
        }
    }
}
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
void MyComponent::pump_mode(std::vector<uint8_t> &pmode)
{
    if (Pump_Mode != pmode)
    {
        this->Pump_Mode = pmode;
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, Pump_Mode[i]);
        
            if (pmode[i] == 1)
            {
                
                Pump_Status[i] = 1;
                pump_total();
            }
        }
    }
}
void MyComponent::pump_reset(std::vector<bool> &pres)
{
    if (Pump_Reset != pres)
    {
        this->Pump_Reset = pres;
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            if (Pump_Reset[i])
            {
                Pump_Total[i][0] = 0;
                Pump_Total[i][1] = 0;
            }
            ESP_LOGD(TAG,"Pump_Total[%d] = %d.%d", i, Pump_Total[i][0], Pump_Total[i][1]);
            ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, (int)Pump_Reset[i]);
        }
    }
}

}  // namespace water_quality
}  // namespace esphome