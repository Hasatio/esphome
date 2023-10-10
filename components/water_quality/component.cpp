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
        ESP_LOGD(TAG,"data1 = %d", *custom_data);
        ESP_LOGD(TAG,"data2 = %d", data2[0]);

delay(1000);
}


}  // namespace water_quality
}  // namespace esphome