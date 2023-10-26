#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <array>
#include <vector>
#include <iterator>

namespace esphome {
namespace water_quality {

class MyComponent : public Component 
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

uint16_t AnIn_TempRes = 1000; //temperature sensor model pt1000 and its resistance is 1k
float AnOut_Vcc, AnOut_Temp, TempRes;
uint8_t DigIn_FilterCoeff[4][10];
uint8_t number;

void setup() override;
void loop() override;

void pump_type(const std::vector<uint8_t> &pt, const uint8_t s)
{
    number = s;
    this->Pump_Type = pt;
}

void pump_calibration(const std::vector<std::vector<uint8_t>> &pc)
{ 
    this->Pump_Calib = pc;
}

void pump_mode(std::vector<uint8_t> &pm)
{
    pm.resize(number);
    this->Pump_Mode = pm;
}

void pump_dose(std::vector<uint8_t> &pd)
{
    pd.resize(number);
    this->Pump_Dose = pd;
}

void AnLIn_Perc(sensor::Sensor *a) 
{
    AnLIn_Perc_ = a;
}

protected:

std::vector<std::vector<uint8_t>> Pump_Calib{};
std::vector<uint8_t> Pump_Type{};
std::vector<uint8_t> Pump_Mode{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Dose{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Circulation{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Reset{0,0,0,0,0,0};
std::vector<uint8_t> Servo_Mode{0,0,0,0,0,0,0,0};
std::vector<uint8_t> Servo_Position{0,0,0,0,0,0,0,0};
std::vector<uint8_t> AnLIn_LvlResMin{};
std::vector<uint8_t> AnLIn_LvlResMax{};
std::vector<uint8_t> AnECIn_Ch{};
std::vector<uint8_t> AnECIn_Type{};
std::vector<uint8_t> AnPHIn_Ch{};
std::vector<uint8_t> AnPHIn_Type{};
std::vector<uint8_t> DigOut_Status{};

sensor::Sensor *Pump_TimeConstant_{nullptr};
sensor::Sensor *Pump_Total_{nullptr};
sensor::Sensor *Pump_Status_{nullptr};
sensor::Sensor *Servo_Status_{nullptr};
sensor::Sensor *AnWTIn_Val_{nullptr};
sensor::Sensor *AnVPowIn_Val_{nullptr};
sensor::Sensor *AnLIn_Perc_{nullptr};
sensor::Sensor *AnGIn_Val_{nullptr};
sensor::Sensor *AnECIn_Val_{nullptr};
sensor::Sensor *AnPHIn_Val_{nullptr};
sensor::Sensor *DigIn_Status_{nullptr};

};


// template<typename... Ts> class PumpModeAction : public Action<Ts...> {
//     public:
//     PumpModeAction(MyComponent *parent) : parent_(parent){};
//     TEMPLATABLE_VALUE(std::vector<uint8_t>, val);
    
//     void set_mode(const std::vector<uint8_t> &set) { val_ = set; }

//     void play(Ts... x) 
//     {
//     std::vector<uint8_t> data = this->val_.value(x...);

//     this->parent_->pump_mode(data);
//     }


//     protected:
//     MyComponent *parent_;
// };
template<typename... Ts> class PumpModeAction : public Action<Ts...> {
    public:
    PumpModeAction(MyComponent *parent) : parent_(parent){};
    TEMPLATABLE_VALUE(std::vector<uint8_t>, mode);
    
    void set_mode(const std::vector<uint8_t> &set) { mode_ = set; }

    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->mode_.value(x...);

    this->parent_->pump_mode(data);
    // this->parent_->pump_mode(this->mode_.value(x...));
    }


    protected:
    MyComponent *parent_;
};

template<typename... Ts> class PumpDoseAction : public Action<Ts...> {
    public:
    PumpDoseAction(MyComponent *parent) : parent_(parent){};
    TEMPLATABLE_VALUE(std::vector<uint8_t>, dose);
    
    void set_dose(const std::vector<uint8_t> &set) { dose_ = set; }

    void play(Ts... x) 
    {
    std::vector<uint8_t> val = this->dose_.value(x...);

    this->parent_->pump_dose(val);
    }


    protected:
    MyComponent *parent_;
};

}  // namespace water_quality
}  // namespace esphome
