#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";



void MyComponent::setup() 
{

}

void MyComponent::loop() 
{

    for (size_t i = 0; i < sizeof Pump_Dose / sizeof Pump_Dose[0]; i++)
    {
        ESP_LOGD(TAG,"x1[%d] = %d", i, Pump_Calib_X1[i]);
        ESP_LOGD(TAG,"y1[%d] = %d", i, Pump_Calib_Y1[i]);
        ESP_LOGD(TAG,"x2[%d] = %d", i, Pump_Calib_X2[i]);
        ESP_LOGD(TAG,"y2[%d] = %d", i, Pump_Calib_Y2[i]);
        ESP_LOGD(TAG,"x3[%d] = %d", i, Pump_Calib_X3[i]);
        ESP_LOGD(TAG,"y3[%d] = %d", i, Pump_Calib_Y3[i]);
        ESP_LOGD(TAG,"x4[%d] = %d", i, Pump_Calib_X4[i]);
        ESP_LOGD(TAG,"y4[%d] = %d", i, Pump_Calib_Y4[i]);

        // ESP_LOGD(TAG,"%d", Pump_Dose[i]);
        ESP_LOGD(TAG,"%d", Pump_Type[i]);
    }
        // ESP_LOGD(TAG,"data[0] = %d", Pump_Dose[0]);
    
        // ESP_LOGD(TAG,"%d", test[0]);
        // ESP_LOGD(TAG,"%d", dd);

delay(1000);
}

}  // namespace water_quality
}  // namespace esphome