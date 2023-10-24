#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";



void MyComponent::setup() 
{
    
}

void MyComponent::loop() 
{
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_X1));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_Y1));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_X2));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_Y2));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_X3));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_Y3));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_X4));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_Y4));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_X5));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_Y5));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_X6));
    // ESP_LOGD(TAG,"%d", sizeof(Pump_Calib_Y6));
    ESP_LOGD(TAG,"%d", sizeof(Pump_Calib));
    ESP_LOGD(TAG,"%d", sizeof(Pump_Calib[0][0]));

    // for (size_t i = 0; i < sizeof(Pump_Calib_X1) / sizeof(Pump_Calib_X1[0]); i++)
    // {
    //     ESP_LOGD(TAG,"x1[%d] = %d", i, Pump_Calib_X1[i]);
    //     ESP_LOGD(TAG,"y1[%d] = %d", i, Pump_Calib_Y1[i]);
    //     // ESP_LOGD(TAG,"x2[%d] = %d", i, Pump_Calib_X2[i]);
    //     // ESP_LOGD(TAG,"y2[%d] = %d", i, Pump_Calib_Y2[i]);
    //     // ESP_LOGD(TAG,"x3[%d] = %d", i, Pump_Calib_X3[i]);
    //     // ESP_LOGD(TAG,"y3[%d] = %d", i, Pump_Calib_Y3[i]);
    //     // ESP_LOGD(TAG,"x4[%d] = %d", i, Pump_Calib_X4[i]);
    //     // ESP_LOGD(TAG,"y4[%d] = %d", i, Pump_Calib_Y4[i]);
    // }

    for (size_t i = 0; i < 8; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            ESP_LOGD(TAG,"%d = %d", i, Pump_Calib[i][j]);
        }
        // ESP_LOGD(TAG,"data[0] = %d", Pump_Dose[0]);
    
        // ESP_LOGD(TAG,"%d", dd);
    }

delay(1000);
}

}  // namespace water_quality
}  // namespace esphome