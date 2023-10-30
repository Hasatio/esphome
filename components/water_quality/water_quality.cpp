#include "water_quality.h"

namespace esphome {
namespace water_quality {

void MyComponent::loop() 
{
    pump_total();
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