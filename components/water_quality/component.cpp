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
    ESP_LOGD(TAG,"%d", Pump_Mode.size());
    ESP_LOGD(TAG,"number: %d", number);

    for (size_t i = 0; i < Pump_Mode.size(); i++)
    {
    
        ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, Pump_Mode[i]);
        ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, Pump_Dose[i]);
    }

    // for (size_t i = 0; i < number*2; i++)
    // {
    //     for (size_t j = 0; j < 4; j++)
    //     {
    //         ESP_LOGD(TAG,"x/y[%d] = %d", i, Pump_Calib[i][j]);
    //     }
    // }

delay(1000);
}

}  // namespace water_quality
}  // namespace esphome