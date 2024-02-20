#include "wq_pump.h"

namespace esphome {
namespace water_quality {

void Pump::Timer_Setup(float period)
{
    float* pump = get_Pump_Time();
    if (timer)
    {
            esp_timer_stop(timer);
            esp_timer_delete(timer);
    }

    // Timer'ı başlat
    esp_timer_create_args_t timer_args = {
        .callback = &Pump::Timer,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = nullptr,
    };
    esp_timer_create(&timer_args, &timer);
        
    esp_timer_start_periodic(timer, static_cast<uint32_t>(period * 1000000));
}
void IRAM_ATTR Pump::Timer(void* arg)
{
    Pump* pumpInstance = static_cast<Pump*>(arg);
    float* pump = pumpInstance->get_Pump_Time();
    pumpInstance->Dosing_Controller(pump);
    pumpInstance->Circulation_Controller(pump);
}
void Pump::Calibration_Status()
{
    float* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* mode = get_Pump_Mode();
    float* dose = get_Pump_Dose();
    float* circ = get_Pump_Circulation();
    uint16_t calib_time = 120;
    bool stat = 0;

    for (size_t i = 0; i < 6; i++)
    {
        if (type[i] > 0 && calib[i] <= 0)
        {
            calib[i] = 1;
            mode[i] = 1;
            if (type[i] == 1)
                dose[i] = calib_time;
            else if (type[i] == 2)
                circ[i] = calib_time;
            stat = 1;
        }
    }
    set_Calibration_Mode(stat);
}

void Pump::Pump_driver(float pwm[])
{     
    uint8_t* stat = get_Pump_Status();
    uint32_t (*tot)[2] = get_Pump_Total();
    bool* reset = get_Pump_Reset();
    float* pump = get_Pump_Time();
    float min = get_Min();
    float min_, mint[6];

    std::copy(pump, pump + 6, mint);
    std::sort(mint, mint + 6);

    for (size_t i = 0; i < 6; ++i) 
    {
        if (mint[i] > 0)
        {
            min_ = mint[i];
            break;
        }
        else
            min_ = 0;
    }
    set_Min(min_);

    if (min != min_)
        Timer_Setup(min_);
    else if (min_ == 0)
    {
        Dosing_Controller(pump);
        Circulation_Controller(pump);
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (pump[i] > 0)
            pwm[i] = 1;
        else
        {
            if (pwm[i])
                ESP_LOGD(TAG, "Pump_Total[%d] = %d.%04d", i, tot[i][0], tot[i][1]);

            pwm[i] = 0;
        }

        if (reset[i])
        {
            tot[i][0] = 0;
            tot[i][1] = 0;
        }
    }
}
void Pump::Dosing_Controller(float pump[])
{
    float* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* mode = get_Pump_Mode();
    uint8_t* stat = get_Pump_Status();
    float* dose = get_Pump_Dose();
    uint32_t (*tot)[2] = get_Pump_Total();
    float min = get_Min();

    for (size_t i = 0; i < 6; i++)
    {
        if (type[i] == 1)
        {
            if (pump[i] > 0)
            {
                if (!get_Calibration_Mode())
                    if (dose[i] > 0)
                    {
                        tot[i][1] = static_cast<uint32_t>(tot[i][1] + calib[i] * min * 10000);
                        tot[i][0] += static_cast<uint32_t>(tot[i][1] / 10000000);
                        if (tot[i][1] >= 10000000)
                            tot[i][1] = 0;
                    }

                dose[i] -= min * calib[i];
            }

            switch (mode[i])
            {
                case 0:
                    if (dose[i] > 0)
                        if (!(i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1)))
                            stat[i] = 0;
                    else
                        stat[i] = 2;
                    break;

                case 1:
                    if (dose[i] > 0)
                        if (i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1))
                        {
                            stat[i] = 1;
                        }
                        else
                            stat[i] = 0;
                    else
                        stat[i] = 2;
                    break;

                case 2:
                    stat[i] = 3;
                    break;
                
                default:
                    break;
            }
            ESP_LOGI(TAG,"dose[%d]: %f", i, dose[i]);

            if (stat[i] == 1)
                if (dose[i] > calib[i])
                    pump[i] = 1;
                else
                    pump[i] = dose[i] / calib[i];
            else
                pump[i] = 0;
        }
    }
}
void Pump::Circulation_Controller(float pump[])
{
    float* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* mode = get_Pump_Mode();
    uint8_t* stat = get_Pump_Status();
    float* circ = get_Pump_Circulation();
    uint32_t (*tot)[2] = get_Pump_Total();
    float min = get_Min();

    for (size_t i = 0; i < 6; i++)
    {
        if (type[i] == 2)
        {
            if (pump[i] > 0)
            {
                if (!get_Calibration_Mode())
                    if (circ[i] > 0)
                    {
                        tot[i][1] = static_cast<uint32_t>(tot[i][1] + calib[i] * min * 10000);
                        tot[i][0] += static_cast<uint32_t>(tot[i][1] / 10000000);
                        if (tot[i][1] >= 10000000)
                            tot[i][1] = 0;
                    }

                circ[i] -= min * calib[i];
            }

            switch (mode[i])
            {
                case 0:
                    if (circ[i] > 0)
                        if (!(i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1)))
                            stat[i] = 0;
                    else
                        stat[i] = 2;
                    break;

                case 1:
                    if (circ[i] > 0)
                        if (i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1))
                        {
                            stat[i] = 1;
                        }
                        else
                            stat[i] = 0;
                    else
                        stat[i] = 2;
                    break;

                case 2:
                    stat[i] = 3;
                    break;
                
                default:
                    break;
            }

            if (stat[i] == 1)
                if (circ[i] > calib[i])
                    pump[i] = 1;
                else
                    pump[i] = circ[i] / calib[i];
            else
                pump[i] = 0;

        }
    }
}

}  // namespace water_quality
}  // namespace esphome