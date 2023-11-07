#include "pump.h"
#include "water_quality.h"

namespace esphome {
namespace water_quality {

    Pump pump;
    MyComponent comp;

static const char *const TAG = "pump";

void MyComponent::pump_calib_gain(const std::vector<float> &pcg)
{
    this->pump.Pump_Calib_Gain = pcg;
}
void MyComponent::pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    pump.dose = d;
    pump.circ = c;
    
    this->pump.Pump_Type = ptype;
}
void MyComponent::pump_dose(std::vector<uint16_t> &pdose)
{
    // pdose.resize(dose);
bool ps[pump.Pump_Type.size()] = {false};

    if (pump.Pump_Dose != pdose)
    {
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            if (pump.Pump_Status[i] != 1)
                ps[i] = true;
            if (pump.Pump_Status[i] == 1 && ps)
            {
                pump.Pump_Dose[i] = pdose[i];
                ps[i] = false;
            }
            else
                pump.Pump_Dose[i] += pdose[i];
            ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pump.Pump_Dose[i]);
        }
    }
}
void MyComponent::pump_circulation(std::vector<uint16_t> &pcirc)
{
    // pcirc.resize(circ);

    if (pump.Pump_Circulation != pcirc)
    {
        this->pump.Pump_Circulation = pcirc;
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pump.Pump_Circulation[i]);
        }
    }
}
void MyComponent::pump_mode(std::vector<uint8_t> &pmode)
{
    if (pump.Pump_Mode != pmode)
    {
        this->pump.Pump_Mode = pmode;
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, pump.Pump_Mode[i]);
        
            if (pmode[i] == 1)
            {
                
                pump.Pump_Status[i] = 1;
                pump.pump_total();
            }
        }
    }
}
void MyComponent::pump_reset(std::vector<bool> &pres)
{
    if (pump.Pump_Reset != pres)
    {
        this->pump.Pump_Reset = pres;
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            if (pump.Pump_Reset[i])
            {
                pump.Pump_Total[i][0] = 0;
                pump.Pump_Total[i][1] = 0;
            }
            ESP_LOGD(TAG,"Pump_Total[%d] = %d.%d", i, pump.Pump_Total[i][0], pump.Pump_Total[i][1]);
            ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, (int)pump.Pump_Reset[i]);
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor
void Pump::sensor()
{
    if (this->comp.Pump_Tot_ != nullptr) { 
        for (size_t i = 0; i < sizeof(comp.Pump_Tot_); i++)
        this->Pump_Tot_->publish_state(Pump_Total[0][i]);
    }

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace water_quality
}  // namespace esphome