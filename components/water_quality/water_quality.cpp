#include "water_quality.h"

namespace esphome {
namespace water_quality {

void MyComponent::loop() 
{
    for (size_t i = 0; i < (dose + circ); i++)
    {
            ESP_LOGD(TAG,"Pump_Total[%d] = %d.%d", i, Pump_Total[0][i], Pump_Total[1][i]);
        
    }
    // delay(1000);
}

void MyComponent::update()
{
    ads1115();
    mcp23008();
    pca9685();
    // sensor();
}

}  // namespace water_quality
}  // namespace esphome