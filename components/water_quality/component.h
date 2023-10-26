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
uint8_t dose, circ;

void setup() override;
void loop() override;

void pump_type(const std::vector<uint8_t> &pt, const uint8_t d, const uint8_t c)
{
    dose = d;
    circ = c;
    this->Pump_Type = pt;
}

void pump_calibration(const std::vector<std::vector<uint8_t>> &pc)
{ 
    this->Pump_Calib = pc;
}

void pump_mode(std::vector<uint8_t> &pm)
{
    pm.resize(dose + circ);
    this->Pump_Mode = pm;
}

void pump_dose(std::vector<uint8_t> &pd)
{
    pd.resize(dose);
    this->Pump_Dose = pd;
}

void pump_circulation(std::vector<uint8_t> &pc)
{
    pc.resize(circ);
    this->Pump_Circulation = pc;
}

void pump_reset(std::vector<uint8_t> &pr)
{
    pr.resize(dose + circ);
    this->Pump_Reset = pr;
}

void servo_channels(std::vector<uint8_t> &sc)
{
    this->Servo_Ch = sc;
}

void servo_mode(std::vector<uint8_t> &sm)
{
    this->Servo_Mode = sm;
}

void servo_position(std::vector<uint8_t> &sp)
{
    this->Servo_Position = sp;
}

void level_res(std::vector<uint8_t> &rmin, std::vector<uint8_t> &rmax)
{
    this->AnInL_LvlResMin = rmin;
    this->AnInL_LvlResMax = rmax;
}

void ec(uint8_t &ch, uint8_t &t)
{
    AnInEC_Ch = ch;
    AnInEC_Type = t;
}

void ph(uint8_t &ch, uint8_t &t)
{
    AnInPH_Ch = ch;
    AnInPH_Type = t;
}

void AnInL_Perc(sensor::Sensor *a) 
{
    AnInL_Perc_ = a;
}

protected:

std::vector<std::vector<uint8_t>> Pump_Calib{};
std::vector<uint8_t> Pump_Type{};
std::vector<uint8_t> Pump_Mode{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Dose{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Circulation{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Reset{0,0,0,0,0,0};
std::vector<uint8_t> Servo_Ch{};
std::vector<uint8_t> Servo_Mode{0,0,0,0,0,0,0,0};
std::vector<uint8_t> Servo_Position{0,0,0,0,0,0,0,0};
std::vector<uint8_t> AnInL_LvlResMin{};
std::vector<uint8_t> AnInL_LvlResMax{};
uint8_t AnInEC_Ch;
uint8_t AnInEC_Type;
uint8_t AnInPH_Ch;
uint8_t AnInPH_Type;
std::vector<uint8_t> DigOut_Status{0,0,0,0};

sensor::Sensor *Pump_TimeConstant_{nullptr};
sensor::Sensor *Pump_Total_{nullptr};
sensor::Sensor *Pump_Status_{nullptr};
sensor::Sensor *Servo_Status_{nullptr};
sensor::Sensor *AnInWT_Val_{nullptr};
sensor::Sensor *AnInVPow_Val_{nullptr};
sensor::Sensor *AnInL_Perc_{nullptr};
sensor::Sensor *AnInG_Val_{nullptr};
sensor::Sensor *AnInEC_Val_{nullptr};
sensor::Sensor *AnInPH_Val_{nullptr};
sensor::Sensor *DigIn_Status_{nullptr};

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
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->pump_dose(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class PumpCirculationAction : public Action<Ts...> {
    public:
    PumpCirculationAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->pump_circulation(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class PumpResetAction : public Action<Ts...> {
    public:
    PumpResetAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->pump_reset(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class ServoModeAction : public Action<Ts...> {
    public:
    ServoModeAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->servo_mode(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

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
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->digital_out(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};

}  // namespace water_quality
}  // namespace esphome
