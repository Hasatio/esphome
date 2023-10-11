#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";

uint8_t data2[] = {11,22,33,44};
void MyComponent::setup() 
{

}

void MyComponent::loop() 
{

    // Pump_CalibX = int(custom_data);
    for (size_t i = 0; i < 8; i++)
    {
        ESP_LOGD(TAG,"x1 = %d", Pump_Calib_X1[i]);
        ESP_LOGD(TAG,"y1 = %d", Pump_Calib_Y1[i]);
        ESP_LOGD(TAG,"x2 = %d", Pump_Calib_X2[i]);
        ESP_LOGD(TAG,"y2 = %d", Pump_Calib_Y2[i]);
        ESP_LOGD(TAG,"x3 = %d", Pump_Calib_X3[i]);
        ESP_LOGD(TAG,"y3 = %d", Pump_Calib_Y3[i]);
        ESP_LOGD(TAG,"x4 = %d", Pump_Calib_X4[i]);
        ESP_LOGD(TAG,"y4 = %d", Pump_Calib_Y4[i]);
    }
    
        

delay(1000);
}


}  // namespace water_quality
}  // namespace esphome