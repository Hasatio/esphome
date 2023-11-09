#pragma once

#ifndef WATER_QUALITY_H
#define WATER_QUALITY_H

#include "mux.h"
#include "analog/analog.h"
#include "digital/digital.h"
#include "pump/pump.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"
#include <array>
#include <vector>
#include <iterator>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

static const char *const TAG = "mycomponent";

namespace esphome {
namespace water_quality {

class MyComponent : public PollingComponent, public i2c::I2CDevice
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

uint16_t PwmFreq = 1000;

    Adafruit_PWMServoDriver pwm;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685
void pca9685_set()
{
    // tcaselect(0);
    Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, Wire);
    
    if (!pwm.begin()) 
    {
      ESP_LOGE(TAG,"Failed to initialize PCA9685.");
    //   while (1);
    }
    /*
    * In theory the internal oscillator (clock) is 25MHz but it really isn't
    * that precise. You can 'calibrate' this by tweaking this number until
    * you get the PWM update frequency you're expecting!
    * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
    * is used for calculating things like writeMicroseconds()
    * Analog servos run at ~50 Hz updates, It is importaint to use an
    * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
    * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
    *    the I2C PCA9685 chip you are setting the value for.
    * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
    *    expected value (50Hz for most ESCs)
    * Setting the value here is specific to each individual I2C PCA9685 chip and
    * affects the calculations for the PWM update frequency. 
    * Failure to correctly set the int.osc value will cause unexpected PWM results
    */
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PwmFreq);
}
void pca9685()
{
    // tcaselect(0);
    // for (uint8_t pin=0; pin<16; pin++) 
    // {
    // pwm.setPWM(pin, 4096, 0);       // turns pin fully on
    // delay(100);
    // pwm.setPWM(pin, 0, 4096);       // turns pin fully off
    // }
    // for (uint16_t i=0; i<4096; i += 8) 
    // {
    //     for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) 
    //     {
    //     pwm.setPWM(pwmnum, 0, (i + (4096/16)*pwmnum) % 4096 );
    //     }
    // }
    // for (uint16_t i=0; i<4096; i += 8) 
    // {
    //     for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) 
    //     {
    //     pwm.setPin(pwmnum, (i + (4096/16)*pwmnum) % 4096 );
    //     }
        
    //     ESP_LOGD(TAG,"pwm = %d", i);
    // }

    // for (uint8_t i = 0; i < Pump_Total[1].size(); i++) 
    //     {
    //         Pump_Total[1][i] += i;
    //     }
}

void sensor();

void setup() override;
void loop() override;
void dump_config() override;
void update() override;

void pump_calib_gain(const std::vector<float> &pcg);
void pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c);
void pump_dose(std::vector<uint16_t> &pdose);
void pump_circulation(std::vector<uint16_t> &pcirc);
void pump_mode(std::vector<uint8_t> &pmode);
void pump_reset(std::vector<bool> &pres);
// void pump_calib_gain(const std::vector<float> &pcg)
// {
//     this->Pump_Calib_Gain = pcg;
// }
// void pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
// {
//     dose = d;
//     circ = c;
    
//     this->Pump_Type = ptype;
// }
// void pump_dose(std::vector<uint16_t> &pdose)
// {
//     // pdose.resize(dose);
// bool ps[Pump_Type.size()] = {false};

//     if (this->Pump_Dose != pdose)
//     {
//         for (size_t i = 0; i < Pump_Type.size(); i++)
//         {
//             if (Pump_Status[i] != 1)
//                 ps[i] = true;
//             if (Pump_Status[i] == 1 && ps)
//             {
//                 Pump_Dose[i] = pdose[i];
//                 ps[i] = false;
//             }
//             else
//                 Pump_Dose[i] += pdose[i];
//             ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, Pump_Dose[i]);
//         }
//     }
// }
// void pump_circulation(std::vector<uint16_t> &pcirc)
// {
//     // pcirc.resize(circ);

//     if (this->Pump_Circulation != pcirc)
//     {
//         this->Pump_Circulation = pcirc;
//         for (size_t i = 0; i < Pump_Type.size(); i++)
//         {
//             ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, Pump_Circulation[i]);
//         }
//     }
// }
// void pump_mode(std::vector<uint8_t> &pmode)
// {
//     if (this->Pump_Mode != pmode)
//     {
//         this->Pump_Mode = pmode;
//         for (size_t i = 0; i < Pump_Type.size(); i++)
//         {
//             ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, Pump_Mode[i]);
        
//             if (pmode[i] == 1)
//             {
                
//                 Pump_Status[i] = 1;
//                 // pump_total();
//             }
//         }
//     }
// }
// void pump_reset(std::vector<bool> &pres)
// {
//     if (this->Pump_Reset != pres)
//     {
//         this->Pump_Reset = pres;
//         for (size_t i = 0; i < Pump_Type.size(); i++)
//         {
//             if (Pump_Reset[i])
//             {
//                 Pump_Total[i][0] = 0;
//                 Pump_Total[i][1] = 0;
//             }
//             ESP_LOGD(TAG,"Pump_Total[%d] = %d.%d", i, Pump_Total[i][0], Pump_Total[i][1]);
//             ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, (int)Pump_Reset[i]);
//         }
//     }
// }

void servo_mode(std::vector<bool> &smode);
void servo_position(std::vector<uint8_t> &spos);
void level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax);
void ec(const uint8_t ch, const uint8_t type);
void ph(const uint8_t ch, const uint8_t type);
void digital_out(std::vector<bool> &dout)
{
    if (DigOut_Status != dout)
    {
        this->DigOut_Status = dout;
        for (size_t i = 0; i < DigOut_Status.size(); i++)
        {
            ESP_LOGD(TAG,"DigOut_Status[%d] = %d", i, (int)DigOut_Status[i]);
        }
    }
}

void Pump_Tot                   (sensor::Sensor *ptot)      { Pump_Tot_ = ptot; }
void Pump_Stat                  (sensor::Sensor *pstat)     { Pump_Stat_ = pstat; }
void Servo_Stat                 (sensor::Sensor *servo)     { Servo_Stat_ = servo; }
void WaterTemp_Sensor_Driver    (sensor::Sensor *wtemp)     { AnInWT_Val_ = wtemp; }
void VPow_Sensor_Driver         (sensor::Sensor *vpow)      { AnInVPow_Val_ = vpow; }
void AnLevel_Sensor_Driver      (sensor::Sensor *level)     { AnInLvl_Perc_ = level; }
void WaterEC_Sensor_Driver      (sensor::Sensor *ec)        { AnInEC_Val_ = ec; }
void WaterPH_Sensor_Driver      (sensor::Sensor *ph)        { AnInPH_Val_ = ph; }
void AnGen_Input_Driver         (sensor::Sensor *a)         { AnInGen_Val_ = a; }
void DigIn_Stat                 (sensor::Sensor *din)       { DigIn_Stat_ = din; }

std::vector<float> Pump_Calib_Gain{0,0,0,0,0,0};
uint8_t dose, circ;
std::vector<uint8_t> Pump_Type{0,0,0,0,0,0};
std::vector<uint16_t> Pump_Dose{0,0,0,0,0,0};
std::vector<uint16_t> Pump_Circulation{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Mode{0,0,0,0,0,0};
std::vector<bool> Pump_Reset{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Status{0,0,0,0,0,0};
std::vector<std::vector<uint16_t>> Pump_Total{{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};

std::vector<bool> Servo_Mode{0,0,0,0,0,0,0,0};
std::vector<uint8_t> Servo_Position{0,0,0,0,0,0,0,0};
std::vector<bool> Servo_Status{0,0,0,0,0,0,0,0};
std::vector<bool> DigOut_Status{0,0,0,0};

protected:
sensor::Sensor *Pump_Tot_{nullptr};
sensor::Sensor *Pump_Stat_{nullptr};
sensor::Sensor *Servo_Stat_{nullptr};
sensor::Sensor *AnInWT_Val_{nullptr};
sensor::Sensor *AnInVPow_Val_{nullptr};
sensor::Sensor *AnInLvl_Perc_{nullptr};
sensor::Sensor *AnInGen_Val_{nullptr};
sensor::Sensor *AnInEC_Val_{nullptr};
sensor::Sensor *AnInPH_Val_{nullptr};
sensor::Sensor *DigIn_Stat_{nullptr};

};

template<typename... Ts> class PumpModeAction : public Action<Ts...> {
    public:
    PumpModeAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->pump_mode(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};
template<typename... Ts> class PumpDoseAction : public Action<Ts...> {
    public:
    PumpDoseAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint16_t> data = this->val_.value(x...);

    this->parent_->pump_dose(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint16_t>, val);

    protected:
    MyComponent *parent_;
};
template<typename... Ts> class PumpCirculationAction : public Action<Ts...> {
    public:
    PumpCirculationAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint16_t> data = this->val_.value(x...);

    this->parent_->pump_circulation(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint16_t>, val);

    protected:
    MyComponent *parent_;
};
template<typename... Ts> class PumpResetAction : public Action<Ts...> {
    public:
    PumpResetAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<bool> data = this->val_.value(x...);

    this->parent_->pump_reset(data);
    }

    TEMPLATABLE_VALUE(std::vector<bool>, val);

    protected:
    MyComponent *parent_;
};
template<typename... Ts> class ServoModeAction : public Action<Ts...> {
    public:
    ServoModeAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<bool> data = this->val_.value(x...);

    this->parent_->servo_mode(data);
    }

    TEMPLATABLE_VALUE(std::vector<bool>, val);

    protected:
    MyComponent *parent_;
};
template<typename... Ts> class ServoPositionAction : public Action<Ts...> {
    public:
    ServoPositionAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->servo_position(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};
template<typename... Ts> class DigitalOutAction : public Action<Ts...> {
    public:
    DigitalOutAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<bool> data = this->val_.value(x...);

    this->parent_->digital_out(data);
    }

    TEMPLATABLE_VALUE(std::vector<bool>, val);

    protected:
    MyComponent *parent_;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WATER_QUALITY_H