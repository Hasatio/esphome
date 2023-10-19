#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>

namespace esphome {
namespace water_quality {

class MyComponent : public Component 
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

uint16_t AnIn_TempRes = 1000; //temperature sensor model pt1000 and its resistance is 1k
float AnOut_Vcc, AnOut_Temp, TempRes;
uint8_t DigIn_FilterCoeff[4][10];
uint8_t dd,dd2[8];

void setup() override;
void loop() override;

void calibration(const std::vector<uint8_t> &x1,
                const std::vector<uint8_t> &y1,
                const std::vector<uint8_t> &x2,
                const std::vector<uint8_t> &y2,
                const std::vector<uint8_t> &x3,
                const std::vector<uint8_t> &y3,
                const std::vector<uint8_t> &x4,
                const std::vector<uint8_t> &y4) 
{ 
    this->Pump_Calib_X1 = x1;
    this->Pump_Calib_Y1 = y1;
    this->Pump_Calib_X2 = x2;
    this->Pump_Calib_Y2 = y2;
    this->Pump_Calib_X3 = x3;
    this->Pump_Calib_Y3 = y3;
    this->Pump_Calib_X4 = x4;
    this->Pump_Calib_Y4 = y4;
}

void pump(const std::vector<uint8_t> &p);
{
    dd2 = p;
}

void dose(uint8_t d)
{
    dd = d;
}

void AnIn_Status(sensor::Sensor *a) 
{
    AnIn_Status_ = a;
}


std::vector<uint8_t> Pump_Calib_X1{};
std::vector<uint8_t> Pump_Calib_Y1{};
std::vector<uint8_t> Pump_Calib_X2{};
std::vector<uint8_t> Pump_Calib_Y2{};
std::vector<uint8_t> Pump_Calib_X3{};
std::vector<uint8_t> Pump_Calib_Y3{};
std::vector<uint8_t> Pump_Calib_X4{};
std::vector<uint8_t> Pump_Calib_Y4{};
std::vector<uint8_t> Pump_Mode{};
std::vector<uint8_t> Pump_Dose{};
std::vector<uint8_t> Pump_Circulation{};
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


// template<typename... Ts> class DoseVolumeAction : public Action<Ts...> {
//  public:
//   DoseVolumeAction(MyComponent *dose) : dose_(dose) {}

//   void play(Ts... x) override { this->dose_->dose(this->data_.value(x...)); }
//   TEMPLATABLE_VALUE(uint8_t, data)

//  protected:
//   MyComponent *dose_;
  
// };

template<typename... Ts> class DoseVolumeAction : public Action<Ts...> {
    public:
    DoseVolumeAction(MyComponent *parent) : parent_(parent){};
    TEMPLATABLE_VALUE(std::vector<uint8_t>, code);
    
    void set_data(const std::vector<uint8_t> &data) { code_ = data; }

    void play(Ts... x) 
    {
    std::vector<uint8_t> msg = this->code_.value(x...);

    this->parent_->pump(msg);
    }


    protected:
    MyComponent *parent_;
};

}  // namespace water_quality
}  // namespace esphome
