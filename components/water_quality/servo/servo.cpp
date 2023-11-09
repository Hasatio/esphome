#include "../water_quality.h"

namespace esphome {
namespace water_quality {

void MyComponent::servo_mode(std::vector<bool> &smode)
{
    if (Servo_Mode != smode)
    {
        this->Servo_Mode = smode;
        for (size_t i = 0; i < Servo_Mode.size(); i++)
        {
            ESP_LOGD(TAG,"Servo_Mode[%d] = %d", i, (int)Servo_Mode[i]);
        }
    }
}
void MyComponent::servo_position(std::vector<uint8_t> &spos)
{
    if (Servo_Position != spos)
    {
        this->Servo_Position = spos;
        for (size_t i = 0; i < Servo_Position.size(); i++)
        {
            ESP_LOGD(TAG,"Servo_Position[%d] = %d", i, Servo_Position[i]);
        }
    }
}

}  // namespace water_quality
}  // namespace esphome