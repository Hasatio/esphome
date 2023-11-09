#include "water_quality.h"

namespace esphome {
namespace water_quality {

// MyComponent::TAG = "mycomponent";

    Mux mux;
    Analog an;
    Digital dig;
    Pump pump;

void MyComponent::setup()
{
    an.ads1115_set();
    dig.mcp23008_set();
    pca9685_set();
}
void MyComponent::dump_config()
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    Wire.begin(SDA,SCL,frq);

    for (uint8_t t=0; t<8; t++) 
    {
      mux.tcaselect(t);
      ESP_LOGI(TAG,"TCA Port %d", t);

      for (uint8_t addr = 0; addr<=127; addr++) 
      {
        if (addr == TCA9548_ADDRESS) continue;

        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) 
        {
          ESP_LOGI(TAG,"Found I2C 0x%x",addr);
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // pump.setdose(16);
    ESP_LOGI(TAG,"Pump_dose = %d", pump.getdose());
    ESP_LOGI(TAG,"dose = %d", pump.dose);
    pump.ddose();
    ESP_LOGI(TAG,"Pump_circ = %d", pump.circ);

    for (size_t i = 0; i < pump.Pump_Type.size(); i++)
    {
        ESP_LOGI(TAG,"Pump_Calib_Gain[%d] = %.2f", i, pump.Pump_Calib_Gain[i]);
    }

    for (size_t i = 0; i < pump.Pump_Type.size(); i++)
    {
        ESP_LOGI(TAG,"Pump_Type[%d] = %d", i, pump.Pump_Type[i]);
        ESP_LOGI(TAG,"Pump_Total[%d] = %d.%d", i, pump.Pump_Total[i][0], pump.Pump_Total[i][1]);
    }

        ESP_LOGI(TAG,"ResMin = %d", an.getAnInLvl_ResMin());
    for (size_t i = 0; i < an.AnInLvl_ResMin.size(); i++)
    {
        ESP_LOGI(TAG,"ResMax[%d] = %d", i, an.AnInLvl_ResMax[i]);
    }

    ESP_LOGI(TAG,"EC_ch = %d", an.AnInEC_Ch);
    ESP_LOGI(TAG,"EC_type = %d", an.AnInEC_Type);
    ESP_LOGI(TAG,"PH_ch = %d", an.AnInPH_Ch);
    ESP_LOGI(TAG,"PH_type = %d", an.AnInPH_Type);
}
void MyComponent::loop() 
{
    // delay(1000);
    // ESP_LOGI(TAG,"Pump_dose = %d", pump.dose);
    // ESP_LOGI(TAG,"Pump_circ = %d", pump.circ);

    // for (size_t i = 0; i < pump.Pump_Type.size(); i++)
    // {
    //     ESP_LOGI(TAG,"Pump_Calib_Gain[%d] = %.2f", i, pump.Pump_Calib_Gain[i]);
    // }

    // for (size_t i = 0; i < pump.Pump_Type.size(); i++)
    // {
    //     ESP_LOGI(TAG,"Pump_Type[%d] = %d", i, pump.Pump_Type[i]);
    //     ESP_LOGI(TAG,"Pump_Total[%d] = %d.%d", i, pump.Pump_Total[i][0], pump.Pump_Total[i][1]);
    // }

    // for (size_t i = 0; i < an.AnInLvl_ResMin.size(); i++)
    // {
    //     ESP_LOGI(TAG,"ResMin[%d] = %d", i, an.AnInLvl_ResMin[i]);
    //     ESP_LOGI(TAG,"ResMax[%d] = %d", i, an.AnInLvl_ResMax[i]);
    // }

    // ESP_LOGI(TAG,"EC_ch = %d", an.AnInEC_Ch);
    // ESP_LOGI(TAG,"EC_type = %d", an.AnInEC_Type);
    // ESP_LOGI(TAG,"PH_ch = %d", an.AnInPH_Ch);
    // ESP_LOGI(TAG,"PH_type = %d", an.AnInPH_Type);
}
void MyComponent::update()
{
    // an.ads1115();
    dig.mcp23008();
    pca9685();
    // pump_total();
    // sensor();
    an.Analog_Input_Driver();
}

void MyComponent::pump_calib_gain(const std::vector<float> &pcg)
{
    this->Pump_Calib_Gain = pcg;
}
void MyComponent::pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    pump.dose = d;
    pump.circ = c;
    
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
            if (pump.Pump_Status[i] != 1)
                ps[i] = true;
            if (pump.Pump_Status[i] == 1 && ps)
            {
                pump.Pump_Dose[i] = pdose[i];
                ps[i] = false;
            }
            else
                Pump_Dose[i] += pdose[i];
            ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pump.Pump_Dose[i]);
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
            ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pump.Pump_Circulation[i]);
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
    if (this->Pump_Reset != pres)
    {
        this->Pump_Reset = pres;
        for (size_t i = 0; i < Pump_Type.size(); i++)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor
void MyComponent::sensor()
{
    if (this->Pump_Tot_ != nullptr) { 
        for (size_t i = 0; i < sizeof(Pump_Tot_); i++)
        this->Pump_Tot_->publish_state(pump.Pump_Total[0][i]);
    }

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace water_quality
}  // namespace esphome