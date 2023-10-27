#include "component.h"

namespace esphome {
namespace water_quality {


void MyComponent::dump_config() 
{
    ESP_LOGI(TAG,"Pump_dose = %d", dose);
    ESP_LOGI(TAG,"Pump_circ = %d", circ);

    for (size_t i = 0; i < (dose + circ)*2; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            ESP_LOGI(TAG,"Pump_Calib[%d]-[%d] = %d", i, j, Pump_Calib[i][j]);
        }
    }

    for (size_t i = 0; i < AnInL_LvlResMin.size(); i++)
    {
        ESP_LOGI(TAG,"ResMin[%d] = %d", i, AnInL_LvlResMin[i]);
        ESP_LOGI(TAG,"ResMax[%d] = %d", i, AnInL_LvlResMax[i]);
    }

    ESP_LOGI(TAG,"EC_ch = %d", AnInEC_Ch);
    ESP_LOGI(TAG,"EC_type = %d", AnInEC_Type);
    ESP_LOGI(TAG,"PH_ch = %d", AnInPH_Ch);
    ESP_LOGI(TAG,"PH_type = %d", AnInPH_Type);
}

void MyComponent::setup() 
{
    
}

void MyComponent::loop() 
{
    delay(1000);
}

}  // namespace water_quality
}  // namespace esphome