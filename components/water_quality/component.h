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

void pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    dose = d;
    circ = c;
    this->Pump_Type = ptype;
}

void pump_calibration(const std::vector<std::vector<uint8_t>> &pcalib)
{ 
    for (size_t i = 0; i < (dose + circ)*2; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            if (Pump_Calib[i][j] != pcalib[i][j])
                ESP_LOGD(TAG,"x[%d]y[%d] = %d", i, j, Pump_Calib[i][j]);
        }
    }
    
    this->Pump_Calib = pcalib;
}

void pump_mode(std::vector<uint8_t> &pmode)
{
    pmode.resize(dose + circ);

    for (size_t i = 0; i < (dose + circ); i++)
    {
        if (Pump_Mode[i] != pmode[i])
            ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, Pump_Mode[i]);
    }

    this->Pump_Mode = pmode;
}

void pump_dose(std::vector<uint8_t> &pdose)
{
    pdose.resize(dose);

    for (size_t i = 0; i < (dose); i++)
    {
        if (Pump_Dose[i] != pdose[i])
            ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, Pump_Dose[i]);
    }

    this->Pump_Dose = pdose;
}

void pump_circulation(std::vector<uint8_t> &pcirc)
{
    pcirc.resize(circ);

    for (size_t i = 0; i < (circ); i++)
    {
        if (Pump_Circulation[i] != pcirc[i])
            ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, Pump_Circulation[i]);
    }

    this->Pump_Circulation = pcirc;
}

void pump_reset(std::vector<uint8_t> &pres)
{
    pres.resize(dose + circ);

    for (size_t i = 0; i < (dose + circ); i++)
    {
        if (Pump_Reset[i] != pres[i])
            ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, Pump_Reset[i]);
    }

    this->Pump_Reset = pres;
}

void servo_mode(std::vector<uint8_t> &smode)
{
    this->Servo_Mode = smode;
}

void servo_position(std::vector<uint8_t> &spos)
{
    this->Servo_Position = spos;
}

void level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax)
{
    this->AnInL_LvlResMin = rmin;
    this->AnInL_LvlResMax = rmax;
}

void ec(const uint8_t ch, const uint8_t type)
{
    AnInEC_Ch = ch;
    AnInEC_Type = type;
}

void ph(const uint8_t ch, const uint8_t type)
{
    AnInPH_Ch = ch;
    AnInPH_Type = type;
}

void digital_out(std::vector<uint8_t> &dout)
{
    this->DigOut_Status = dout;
}

void AnInL_Perc(sensor::Sensor *level) 
{
    AnInL_Perc_ = level;
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
std::vector<uint16_t> AnInL_LvlResMin{};
std::vector<uint16_t> AnInL_LvlResMax{};
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
