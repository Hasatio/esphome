#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
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
uint8_t dd,dd2[8]={0,0,0,0,0,0,0,0};
int ttt;

void setup() override;
void loop() override;

void pump_calibration(const std::vector<std::vector<uint8_t>> &pc)
{ 
    so = sizeof(pc);
    so2 = sizeof(pc[0]);
    // for (size_t i = 0; i < 8; i++)
    // {
    //     for (int j = 0; j < 8; j++)
    //     {
    //         this->Pump_Calib[i][j] = pc[i][j];
    //     }
    // }
}

void pump_type(const std::vector<uint8_t> &pt)
{
    so3 = sizeof(pt);
    so4 = sizeof(pt[0]);
    // for (int i = 0; i < sizeof(Pump_Type) / sizeof(Pump_Type[0]); i++)
    // {
    //     this->Pump_Type[i] = pt[i];
    // }
}

void pump_dose(const std::vector<uint8_t> &d)
{
    this->Pump_Dose = d;
}

void AnIn_Status(sensor::Sensor *a) 
{
    AnIn_Status_ = a;
}

uint8_t so,so2,so3,so4;
uint8_t Pump_Calib[12][8];
uint8_t Pump_Type[6];
std::vector<uint8_t> Pump_Mode{};
std::vector<uint8_t> Pump_Dose{0};
std::vector<uint8_t> Pump_Circulation{0};
std::vector<uint8_t> Pump_Reset{};
std::vector<uint8_t> Servo_Mode{};
std::vector<uint8_t> Servo_Position{};
std::vector<uint8_t> AnIn_LvlResMin{};
std::vector<uint8_t> AnIn_LvlResMax{};
std::vector<uint8_t> DigIn_Status{};

protected:

sensor::Sensor *Pump_TimeConstant_{nullptr};
sensor::Sensor *Pump_Total_{nullptr};
sensor::Sensor *Pump_Status_{nullptr};
sensor::Sensor *Servo_Status_{nullptr};
sensor::Sensor *AnIn_Status_{nullptr};
sensor::Sensor *AnIn_SensPerc_{nullptr};
sensor::Sensor *DigOut_Status_{nullptr};

};


// template<typename... Ts> class PumpTypeAction : public Action<Ts...> {
//     public:
//     PumpDoseAction(MyComponent *parent) : parent_(parent){};
//     TEMPLATABLE_VALUE(std::vector<uint8_t>, type);
    
//     void set_type(const std::vector<uint8_t> &set) { type_ = set; }

//     void play(Ts... x) 
//     {
//     std::vector<uint8_t> data = this->type_.value(x...);

//     this->parent_->pump_type(data);
//     }


//     protected:
//     MyComponent *parent_;
// };

template<typename... Ts> class PumpDoseAction : public Action<Ts...> {
    public:
    PumpDoseAction(MyComponent *parent) : parent_(parent){};
    TEMPLATABLE_VALUE(std::vector<uint8_t>, dose);
    
    void set_dose(const std::vector<uint8_t> &set) { dose_ = set; }

    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->dose_.value(x...);

    this->parent_->pump_dose(data);
    }


    protected:
    MyComponent *parent_;
};

}  // namespace water_quality
}  // namespace esphome
