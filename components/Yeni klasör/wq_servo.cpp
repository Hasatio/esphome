// #include "water_quality.h"
#include "wq_servo.h"

namespace esphome {
namespace water_quality {

<<<<<<< HEAD
void Servo::Servo_driver(float pwm[])
{
    Servo_Controller(pwm);
}
void Servo::Servo_Controller(float perc[])
{
    bool* mode = get_Servo_Mode();
    uint8_t* pos = get_Servo_Position();
    bool* stat = get_Servo_Status();

    for (size_t i = 0; i < 8; i++)
    {
        if (mode[i] == 1)
        {
            perc[i + 8] = static_cast<float>(pos[i]) / 100;
            stat[i] = 1;

            ESP_LOGI(TAG,"servo[%d]: %f", i, perc[i + 8]);
        }
        else
        {
            stat[i] = 0;
        }
    }
}

=======
>>>>>>> parent of 3b0422f (pump and whole installation)
}  // namespace water_quality
}  // namespace esphome