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
    uint8_t* model = get_Pump_Model();
    uint8_t* mode = get_Pump_Mode();
    float* dose = get_Pump_Dose();
    float* circ = get_Pump_Circulation();
    uint16_t calib_time = 120;
    bool stat = 0;

    for (size_t i = 0; i < 6; i++)
    {
        if (calib[i] == 0 && type[i] > 0 && model[i] == 1)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GENERIC
void Pump::Generic_Pump_Driver(float pwm[])
{
    uint32_t (*tot)[2] = get_Pump_Total();
    bool* reset = get_Pump_Reset();
    float* pump = get_Pump_Time();
    float min = get_Min();
    float min_ = 0, mint[6];

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
        else if (pump[i] < 0)
            pwm[i] = 0;
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
    uint8_t* model = get_Pump_Model();
    uint8_t* mode = get_Pump_Mode();
    uint8_t* stat = get_Pump_Status();
    float* dose = get_Pump_Dose();
    uint32_t (*tot)[2] = get_Pump_Total();
    float min = get_Min();

    for (size_t i = 0; i < 6; i++)
    {
        if (type[i] == 1)
            if (model[i] > 0)
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

                    dose[i] = fabs(dose[i] - min * calib[i]);
                }

                switch (mode[i])
                {
                    case 0:
                        if (dose[i] > 0)
                            if (!(i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1)))
                                stat[i] = 0;
                        else if (dose[i] == 0)
                            stat[i] = 2;
                        break;

                    case 1:
                        if (dose[i] > 0)
                            if (i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1))
                                stat[i] = 1;
                            else
                                stat[i] = 0;
                        else if (dose[i] == 0)
                            stat[i] = 2;
                        break;

                    case 2:
                        stat[i] = 3;
                        break;
                    
                    default:
                        break;
                }

                if (stat[i] == 1)
                    if (dose[i] > calib[i])
                        pump[i] = 1;
                    else
                        pump[i] = dose[i] / calib[i];
                else
                    pump[i] = 0;
                    
                if (model[i] == 2)
                    pump[i] *= -calib[i];
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

                circ[i] = fabs(circ[i] - min * calib[i]);
            }

            switch (mode[i])
            {
                case 0:
                    if (circ[i] > 0)
                        if (!(i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1)))
                            stat[i] = 0;
                    else if (circ[i] == 0)
                        stat[i] = 2;
                    break;

                case 1:
                    if (circ[i] > 0)
                        if (i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1))
                            stat[i] = 1;
                        else
                            stat[i] = 0;
                    else if (circ[i] == 0)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SERIAL
double Pump::Serial_Com_Pump_Driver()
{
    float* calib = get_Pump_Calib_Gain();
    uint8_t* type = get_Pump_Type();
    uint8_t* model = get_Pump_Model();
    uint8_t* mode = get_Pump_Mode();
    uint8_t* stat = get_Pump_Status();
    float* dose = get_Pump_Dose();
    uint32_t (*tot)[2] = get_Pump_Total();
    float min = get_Min();
    double vol = 0;

    for (size_t i = 0; i < 6; i++)
    {
        if (type[i] == 1)
            if (model[i] == 2)
            {
                ESP_LOGD(TAG,"here");
                // if (pump[i] > 0)
                // {
                //     if (!get_Calibration_Mode())
                //         if (dose[i] > 0)
                //         {
                //             tot[i][1] = static_cast<uint32_t>(tot[i][1] + calib[i] * min * 10000);
                //             tot[i][0] += static_cast<uint32_t>(tot[i][1] / 10000000);
                //             if (tot[i][1] >= 10000000)
                //                 tot[i][1] = 0;
                //         }

                //     dose[i] = fabs(dose[i] - min * calib[i]);
                // }

                switch (mode[i])
                {
                    case 0:
                        if (dose[i] == 0)
                            stat[i] = 2;
                        break;

                    case 1:
                        if (dose[i] > 0)
                            if (i % 2 == 0 || (i % 2 == 1 && stat[i - 1] != 1))
                                stat[i] = 1;
                            else
                                stat[i] = 0;
                        else if (dose[i] == 0)
                            stat[i] = 2;
                        break;

                    case 2:
                        stat[i] = 3;
                        break;
                    
                    default:
                        break;
                }

                if (stat[i] == 1)
                    vol = dose[i];
                else
                    vol = 0;
                    
                // if (model[i] == 2)
                //     pump[i] *= -calib[i];
            }
    }
return vol;
}

void Pump::find() { this->queue_command_(EZO_PMP_COMMAND_FIND, 0, 0, true); }
void Pump::dose_continuously()
{
    this->queue_command_(EZO_PMP_COMMAND_DOSE_CONTINUOUSLY, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}
void Pump::dose_volume(double volume)
{
    if (this->volume_ != volume)
    {
        ESP_LOGI(TAG,"volume = %f", volume);
        this->volume_ == volume;
        this->queue_command_(EZO_PMP_COMMAND_DOSE_VOLUME, volume, 0, true);
        this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
        this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
    }
}
void Pump::dose_volume_over_time(double volume, int duration)
{
    this->queue_command_(EZO_PMP_COMMAND_DOSE_VOLUME_OVER_TIME, volume, duration, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}
void Pump::dose_with_constant_flow_rate(double volume, int duration)
{
    this->queue_command_(EZO_PMP_COMMAND_DOSE_WITH_CONSTANT_FLOW_RATE, volume, duration, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}
void Pump::set_calibration_volume(double volume)
{
    this->queue_command_(EZO_PMP_COMMAND_SET_CALIBRATION_VOLUME, volume, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_CALIBRATION_STATUS, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_MAX_FLOW_RATE, 0, 0, true);
}
void Pump::clear_total_volume_dosed()
{
    this->queue_command_(EZO_PMP_COMMAND_CLEAR_TOTAL_VOLUME_DOSED, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_TOTAL_VOLUME_DOSED, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_ABSOLUTE_TOTAL_VOLUME_DOSED, 0, 0, true);
}
void Pump::clear_calibration()
{
    this->queue_command_(EZO_PMP_COMMAND_CLEAR_CALIBRATION, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_CALIBRATION_STATUS, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_MAX_FLOW_RATE, 0, 0, true);
}
void Pump::pause_dosing()
{
    this->queue_command_(EZO_PMP_COMMAND_PAUSE_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_PAUSE_STATUS, 0, 0, true);
}
void Pump::stop_dosing() { this->queue_command_(EZO_PMP_COMMAND_STOP_DOSING, 0, 0, true); }
void Pump::change_i2c_address(int address)
{
    this->queue_command_(EZO_PMP_COMMAND_CHANGE_I2C_ADDRESS, 0, address, true);
}
void Pump::exec_arbitrary_command(const std::basic_string<char> &command)
{
    this->arbitrary_command_ = command.c_str();
    this->queue_command_(EZO_PMP_COMMAND_EXEC_ARBITRARY_COMMAND_ADDRESS, 0, 0, true);
}
void Pump::clear_current_command_()
{
    this->current_command_ = EZO_PMP_COMMAND_NONE;
    this->is_waiting_ = false;
}
void Pump::queue_command_(uint16_t command, double volume, int duration, bool should_schedule)
{
    if (!should_schedule)
        return;

    if (this->next_command_queue_length_ >= 10)
    {
        ESP_LOGE(TAG, "Tried to queue command '%d' but queue is full", command);
        return;
    }

    this->next_command_queue_[this->next_command_queue_last_] = command;
    this->next_command_volume_queue_[this->next_command_queue_last_] = volume;
    this->next_command_duration_queue_[this->next_command_queue_last_] = duration;

    ESP_LOGV(TAG, "Queue command '%d' in position '%d'", command, next_command_queue_last_);

    // Move positions
    next_command_queue_last_++;
    if (next_command_queue_last_ >= 10)
        next_command_queue_last_ = 0;

    next_command_queue_length_++;
}
void Pump::pop_next_command_()
{
    if (this->next_command_queue_length_ <= 0)
    {
        ESP_LOGE(TAG, "Tried to dequeue command from empty queue");
        this->next_command_ = EZO_PMP_COMMAND_NONE;
        this->next_command_volume_ = 0;
        this->next_command_duration_ = 0;
        return;
    }

    // Read from Head
    this->next_command_ = this->next_command_queue_[this->next_command_queue_head_];
    this->next_command_volume_ = this->next_command_volume_queue_[this->next_command_queue_head_];
    this->next_command_duration_ = this->next_command_duration_queue_[this->next_command_queue_head_];

    // Move positions
    next_command_queue_head_++;
    if (next_command_queue_head_ >= 10)
        next_command_queue_head_ = 0;

    next_command_queue_length_--;
}
uint16_t Pump::peek_next_command_()
{
    if (this->next_command_queue_length_ <= 0)
        return EZO_PMP_COMMAND_NONE;

    return this->next_command_queue_[this->next_command_queue_head_];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace water_quality
}  // namespace esphome