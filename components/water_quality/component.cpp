#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";



void MyComponent::setup() 
{
    
}

void MyComponent::loop() 
{
    delay(1000);
}

}  // namespace water_quality
}  // namespace esphome