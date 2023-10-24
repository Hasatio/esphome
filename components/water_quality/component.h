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

// void pump_calibration(const std::vector<uint8_t> &x1,
    //                   const std::vector<uint8_t> &y1,
    //                   const std::vector<uint8_t> &x2,
    //                   const std::vector<uint8_t> &y2,
    //                   const std::vector<uint8_t> &x3,
    //                   const std::vector<uint8_t> &y3,
    //                   const std::vector<uint8_t> &x4,
    //                   const std::vector<uint8_t> &y4) 
// { 
//     this->Pump_Calib_X1 = x1;
//     this->Pump_Calib_Y1 = y1;
//     this->Pump_Calib_X2 = x2;
//     this->Pump_Calib_Y2 = y2;
//     this->Pump_Calib_X3 = x3;
//     this->Pump_Calib_Y3 = y3;
//     this->Pump_Calib_X4 = x4;
//     this->Pump_Calib_Y4 = y4;
// }
void pump_calibration(const std::vector<std::vector<uint8_t>> &x) //uint8_t x[][]
{ 
    // x.resize(8);
    // y.resize(8);
    // this->Pump_Calib_X1 = x;
    // this->Pump_Calib_Y1 = y;
    this->Pump_Calib = x;
}

void pump_type(uint8_t t[])
{
    for (int i = 0; i < 8; i++)
    {
        Pump_Type[i] = t[i];
    }
}

void pump_dose(const std::vector<uint8_t> &d)
{
    this->Pump_Dose = d;
}

void AnIn_Status(sensor::Sensor *a) 
{
    AnIn_Status_ = a;
}


// uint8_t Pump_Calib[][8];
std::vector<std::vector<uint8_t>> Pump_Calib{{0},{0}};
std::vector<uint8_t> Pump_Calib_X1{0};
std::vector<uint8_t> Pump_Calib_Y1{0};
std::vector<uint8_t> Pump_Calib_X2{};
std::vector<uint8_t> Pump_Calib_Y2{};
std::vector<uint8_t> Pump_Calib_X3{};
std::vector<uint8_t> Pump_Calib_Y3{};
std::vector<uint8_t> Pump_Calib_X4{};
std::vector<uint8_t> Pump_Calib_Y4{};
std::vector<uint8_t> Pump_Calib_X5{};
std::vector<uint8_t> Pump_Calib_Y5{};
std::vector<uint8_t> Pump_Calib_X6{};
std::vector<uint8_t> Pump_Calib_Y6{};
uint8_t Pump_Type[];
// std::vector<uint8_t> Pump_Type{0};
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
