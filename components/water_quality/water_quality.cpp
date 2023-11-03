#include "mux.h"
#include "analog.h"
#include "water_quality.h"

namespace esphome {
namespace water_quality {

void MyComponent::dump_config() override
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    Wire.begin(SDA,SCL,freq);

    for (uint8_t t=0; t<8; t++) 
    {
      tcaselect(t);
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

    ESP_LOGI(TAG,"Pump_dose = %d", dose);
    ESP_LOGI(TAG,"Pump_circ = %d", circ);

    for (size_t i = 0; i < Pump_Type.size(); i++)
    {
        ESP_LOGI(TAG,"Pump_Calib_Gain[%d] = %d", i, Pump_Calib_Gain[i]);
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
}

}  // namespace water_quality
}  // namespace esphome