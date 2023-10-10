#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";

void MyComponent::setup() 
{

}

void MyComponent::loop() 
{

    // Pump_CalibX = int(custom_data);
        ESP_LOGD(TAG,"data = %d", custom_data);

delay(1000);
}


}  // namespace water_quality
}  // namespace esphome