#include "water_quality.h"
#include "pump.h"

namespace esphome {
namespace water_quality {

    Pump pum;

void MyComponent::pump_calib_gain(const std::vector<float> &pcg)
{
    this->Pump_Calib_Gain = pcg;
}
void MyComponent::pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    pum.dose = 2;
    pum.circ = c;
    
    this->Pump_Type = ptype;
}
void MyComponent::pump_dose(std::vector<uint16_t> &pdose)
{
    // pdose.resize(dose);
bool ps[Pump_Type.size()] = {false};

    if (this->Pump_Dose != pdose)
    {
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            if (pum.Pump_Status[i] != 1)
                ps[i] = true;
            if (pum.Pump_Status[i] == 1 && ps)
            {
                pum.Pump_Dose[i] = pdose[i];
                ps[i] = false;
            }
            else
                Pump_Dose[i] += pdose[i];
            ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pum.Pump_Dose[i]);
        }
    }
}
void MyComponent::pump_circulation(std::vector<uint16_t> &pcirc)
{
    // pcirc.resize(circ);

    if (this->Pump_Circulation != pcirc)
    {
        this->Pump_Circulation = pcirc;
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pum.Pump_Circulation[i]);
        }
    }
}
void MyComponent::pump_mode(std::vector<uint8_t> &pmode)
{
    if (this->Pump_Mode != pmode)
    {
        this->Pump_Mode = pmode;
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, pum.Pump_Mode[i]);
        
            if (pmode[i] == 1)
            {
                
                pum.Pump_Status[i] = 1;
                pum.pump_total();
            }
        }
    }
}
void MyComponent::pump_reset(std::vector<bool> &pres)
{
    if (this->Pump_Reset != pres)
    {
        this->Pump_Reset = pres;
        for (size_t i = 0; i < Pump_Type.size(); i++)
        {
            if (pum.Pump_Reset[i])
            {
                pum.Pump_Total[i][0] = 0;
                pum.Pump_Total[i][1] = 0;
            }
            ESP_LOGD(TAG,"Pump_Total[%d] = %d.%d", i, pum.Pump_Total[i][0], pum.Pump_Total[i][1]);
            ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, (int)pum.Pump_Reset[i]);
        }
    }
}

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