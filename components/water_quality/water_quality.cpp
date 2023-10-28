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
    if (this->DigOut_3_Status_ != nullptr) this->DigOut_3_Status_->publish_state(DigOut_Status[3]);
}

}  // namespace water_quality
}  // namespace esphome