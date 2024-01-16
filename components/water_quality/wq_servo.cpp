#include "wq_servo.h"

namespace esphome {
namespace water_quality {

void Servo::Servo_driver(float pwm[])
{
    Servo_Controller(pwm);
}
void Servo::Servo_Controller(float perc[])
{
    bool* mode = get_Servo_Mode();
    uint8_t* pos = get_Servo_Position();
    bool* stat = get_Servo_Status();

    for (size_t i = 8; i < 16; i++)
    {
        if (mode[i] == 1)
        {
            perc[i] = pos[i] / 100;
            stat[i] = 1;
        }
        else
        {
            stat[i] = 0;
        }
        ESP_LOGI(TAG,"pwm: %f", perc[i]);
    }
}

}  // namespace water_quality
}  // namespace esphome