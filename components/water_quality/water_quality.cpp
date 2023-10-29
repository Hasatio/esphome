#include "water_quality.h"

namespace esphome {
namespace water_quality {

void MyComponent::setup() 
{
    
}

void MyComponent::loop() 
{
    
}

void MyComponent::update() 
{
    if (this->AnInVPow_Val_ != nullptr) 
    {
        this->AnInVPow_Val_->publish_state(AnOut_Vcc);
    }
}

}  // namespace water_quality
}  // namespace esphome