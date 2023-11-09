#include "water_quality.h"

namespace esphome {
namespace water_quality {

// MyComponent::TAG = "mycomponent";

    Mux mux;
    // Analog an;
    Digital dig;
    MyComponent pump;

void MyComponent::setup()
{
    // an.ads1115_set();
    dig.mcp23008_set();
    pca9685_set();
}

void MyComponent::dump_config()
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    // Wire.begin(SDA,SCL,frq);

    // for (uint8_t t=0; t<8; t++) 
    // {
    //   mux.tcaselect(t);
    //   ESP_LOGI(TAG,"TCA Port %d", t);

    //   for (uint8_t addr = 0; addr<=127; addr++) 
    //   {
    //     if (addr == TCA9548_ADDRESS) continue;

    //     Wire.beginTransmission(addr);
    //     if (!Wire.endTransmission()) 
    //     {
    //       ESP_LOGI(TAG,"Found I2C 0x%x",addr);
    //     }
    //   }
    // }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ESP_LOGI(TAG,"Pump_dose = %d", dose);
    ESP_LOGI(TAG,"Pump_circ = %d", circ);

    for (size_t i = 0; i < Pump_Type.size(); i++)
    {
        ESP_LOGI(TAG,"Pump_Calib_Gain[%d] = %.2f", i, Pump_Calib_Gain[i]);
    }

    for (size_t i = 0; i < Pump_Type.size(); i++)
    {
        ESP_LOGI(TAG,"Pump_Type[%d] = %d", i, Pump_Type[i]);
        ESP_LOGI(TAG,"Pump_Total[%d] = %d.%d", i, Pump_Total[i][0], Pump_Total[i][1]);
    }
    
    for (size_t i = 0; i < AnInLvl_ResMin.size(); i++)
    {
        ESP_LOGI(TAG,"ResMin[%d] = %d", i, AnInLvl_ResMin[i]);
        ESP_LOGI(TAG,"ResMax[%d] = %d", i, AnInLvl_ResMax[i]);
    }

    ESP_LOGI(TAG,"EC_ch = %d", AnInEC_Ch);
    ESP_LOGI(TAG,"EC_type = %d", AnInEC_Type);
    ESP_LOGI(TAG,"PH_ch = %d", AnInPH_Ch);
    ESP_LOGI(TAG,"PH_type = %d", AnInPH_Type);
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