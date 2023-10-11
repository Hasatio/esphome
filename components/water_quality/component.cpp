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
        ESP_LOGD(TAG,"x = %d", Pump_Calib_X[i]);
        ESP_LOGD(TAG,"y = %d", Pump_Calib_Y[i]);
    }
    
        

delay(1000);
}


}  // namespace water_quality
}  // namespace esphome