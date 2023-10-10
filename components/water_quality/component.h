#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include "Adafruit_MCP23X08.h"
#include <Adafruit_PWMServoDriver.h>

#include <vector>

namespace esphome {
namespace water_quality {

class MyComponent : public Component 
{
public:

// float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void setup() override;
void loop() override;

void tcaselect(uint8_t bus);

void pump(String PT[6],uint8_t PCX[8],uint8_t PCY[8],uint8_t PM[4],uint8_t PD[4]);

void set_user_defined_char(uint8_t pos, const std::vector<uint8_t> &data) { this->user_defined_chars_[pos] = data; }

void Pump_0_Total(sensor::Sensor *p) {Pump_0_Total_ = p;}
void Pump_1_Total(sensor::Sensor *p) {Pump_1_Total_ = p;}
void Pump_2_Total(sensor::Sensor *p) {Pump_2_Total_ = p;}
void Pump_3_Total(sensor::Sensor *p) {Pump_3_Total_ = p;}
void Pump_0_Status(sensor::Sensor *p) {Pump_0_Status_ = p;}
void Pump_1_Status(sensor::Sensor *p) {Pump_1_Status_ = p;}
void Pump_2_Status(sensor::Sensor *p) {Pump_2_Status_ = p;}
void Pump_3_Status(sensor::Sensor *p) {Pump_3_Status_ = p;}

// void Servo_0_Position(sensor::Sensor *s) {Servo_0_Position = s;}
// void Servo_1_Position(sensor::Sensor *s) {Servo_1_Position = s;}
// void Servo_2_Position(sensor::Sensor *s) {Servo_2_Position = s;}
// void Servo_3_Position(sensor::Sensor *s) {Servo_3_Position = s;}
// void Servo_4_Position(sensor::Sensor *s) {Servo_4_Position = s;}
// void Servo_5_Position(sensor::Sensor *s) {Servo_5_Position = s;}
// void Servo_6_Position(sensor::Sensor *s) {Servo_6_Position = s;}
// void Servo_7_Position(sensor::Sensor *s) {Servo_7_Position = s;}
// void Servo_0_Status(sensor::Sensor *s) {Servo_0_Status = s;}
// void Servo_1_Status(sensor::Sensor *s) {Servo_1_Status = s;}
// void Servo_2_Status(sensor::Sensor *s) {Servo_2_Status = s;}
// void Servo_3_Status(sensor::Sensor *s) {Servo_3_Status = s;}
// void Servo_4_Status(sensor::Sensor *s) {Servo_4_Status = s;}
// void Servo_5_Status(sensor::Sensor *s) {Servo_5_Status = s;}
// void Servo_6_Status(sensor::Sensor *s) {Servo_6_Status = s;}
// void Servo_7_Status(sensor::Sensor *s) {Servo_7_Status = s;}

void AnOut_Status(sensor::Sensor *a) 
{
    AnOut_Status_ = a;
}
// void AnOut_0_SensPerc_(sensor::Sensor *a) {AnOut_0_SensPerc_ = a;}
// void AnOut_1_SensPerc_(sensor::Sensor *a) {AnOut_1_SensPerc_ = a;}
// void AnOut_2_SensPerc_(sensor::Sensor *a) {AnOut_2_SensPerc_ = a;}
// void AnOut_3_SensPerc_(sensor::Sensor *a) {AnOut_3_SensPerc_ = a;}

// void DigIn_0_Status_(sensor::Sensor *d) {DigIn_0_Status_ = d;}
// void DigIn_1_Status_(sensor::Sensor *d) {DigIn_1_Status_ = d;}
// void DigIn_2_Status_(sensor::Sensor *d) {DigIn_2_Status_ = d;}
// void DigIn_3_Status_(sensor::Sensor *d) {DigIn_3_Status_ = d;}
// void DigOut_0_Status_(sensor::Sensor *d) {DigOut_0_Status_ = d;}
// void DigOut_1_Status_(sensor::Sensor *d) {DigOut_1_Status_ = d;}
// void DigOut_2_Status_(sensor::Sensor *d) {DigOut_2_Status_ = d;}
// void DigOut_3_Status_(sensor::Sensor *d) {DigOut_3_Status_ = d;}

protected:

std::map<uint8_t, std::vector<uint8_t> > user_defined_chars_;

sensor::Sensor *Pump_0_Total_{nullptr};
sensor::Sensor *Pump_1_Total_{nullptr};
sensor::Sensor *Pump_2_Total_{nullptr};
sensor::Sensor *Pump_3_Total_{nullptr};
sensor::Sensor *Pump_0_Status_{nullptr};
sensor::Sensor *Pump_1_Status_{nullptr};
sensor::Sensor *Pump_2_Status_{nullptr};
sensor::Sensor *Pump_3_Status_{nullptr};

sensor::Sensor *Servo_0_Position{nullptr};
sensor::Sensor *Servo_1_Position{nullptr};
sensor::Sensor *Servo_2_Position{nullptr};
sensor::Sensor *Servo_3_Position{nullptr};
sensor::Sensor *Servo_4_Position{nullptr};
sensor::Sensor *Servo_5_Position{nullptr};
sensor::Sensor *Servo_6_Position{nullptr};
sensor::Sensor *Servo_7_Position{nullptr};
sensor::Sensor *Servo_0_Status{nullptr};
sensor::Sensor *Servo_1_Status{nullptr};
sensor::Sensor *Servo_2_Status{nullptr};
sensor::Sensor *Servo_3_Status{nullptr};
sensor::Sensor *Servo_4_Status{nullptr};
sensor::Sensor *Servo_5_Status{nullptr};
sensor::Sensor *Servo_6_Status{nullptr};
sensor::Sensor *Servo_7_Status{nullptr};

sensor::Sensor *AnOut_Status_{nullptr};
// sensor::Sensor *AnOut_0_Temp_{nullptr};
// sensor::Sensor *AnOut_1_Vcc_{nullptr};
// sensor::Sensor *AnOut_2_LvlPerc_{nullptr};
// sensor::Sensor *AnOut_3_LvlPerc_{nullptr};
sensor::Sensor *AnOut_0_SensPerc_{nullptr};
sensor::Sensor *AnOut_1_SensPerc_{nullptr};
sensor::Sensor *AnOut_2_SensPerc_{nullptr};
sensor::Sensor *AnOut_3_SensPerc_{nullptr};

sensor::Sensor *DigIn_0_Status_{nullptr};
sensor::Sensor *DigIn_1_Status_{nullptr};
sensor::Sensor *DigIn_2_Status_{nullptr};
sensor::Sensor *DigIn_3_Status_{nullptr};
sensor::Sensor *DigOut_0_Status_{nullptr};
sensor::Sensor *DigOut_1_Status_{nullptr};
sensor::Sensor *DigOut_2_Status_{nullptr};
sensor::Sensor *DigOut_3_Status_{nullptr};

};
 
}  // namespace water_quality
}  // namespace esphome
