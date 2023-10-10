#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";

uint8_t data[8];
uint8_t data2[] = {1,2,3,4};
void MyComponent::setup() 
{

}

void MyComponent::loop() 
{

    // Pump_CalibX = int(custom_data);
        ESP_LOGD(TAG,"data = %d", custom_data);
        ESP_LOGD(TAG,"data2 = %d", data2);

delay(1000);
}


}  // namespace water_quality
}  // namespace esphome