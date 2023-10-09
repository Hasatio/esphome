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

void dat(std::vector<uint8_t> &data);

void pt(sensor::Sensor *Pump_0_Total) {Pump_0_Total_ = Pump_0_Total;};
void Pump_1_Total(sensor::Sensor *pt) {Pump_1_Total_ = pt;}
void Pump_2_Total(sensor::Sensor *pt) {Pump_2_Total_ = pt;}
void Pump_3_Total(sensor::Sensor *pt) {Pump_3_Total_ = pt;}

void ps(sensor::Sensor *Pump_0_Status) {Pump_0_Status_ = Pump_0_Status;};
void Pump_1_Status(sensor::Sensor *ps) {Pump_1_Status_ = ps;}
void Pump_2_Status(sensor::Sensor *ps) {Pump_2_Status_ = ps;}
void Pump_3_Status(sensor::Sensor *ps) {Pump_3_Status_ = ps;}

// void Pump_1_Status(sensor::Sensor *ps) {Servo_0_Position = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {Servo_1_Position = ps;}
// void Pump_2_Status(sensor::Sensor *ps) {Servo_2_Position = ps;}
// void Pump_3_Status(sensor::Sensor *ps) {Servo_3_Position = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {Servo_4_Position = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {Servo_5_Position = ps;}
// void Pump_2_Status(sensor::Sensor *ps) {Servo_6_Position = ps;}
// void Pump_3_Status(sensor::Sensor *ps) {Servo_7_Position = ps;}

// void Pump_1_Status(sensor::Sensor *ps) {Servo_0_Status = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {Servo_1_Status = ps;}
// void Pump_2_Status(sensor::Sensor *ps) {Servo_2_Status = ps;}
// void Pump_3_Status(sensor::Sensor *ps) {Servo_3_Status = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {Servo_4_Status = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {Servo_5_Status = ps;}
// void Pump_2_Status(sensor::Sensor *ps) {Servo_6_Status = ps;}
// void Pump_3_Status(sensor::Sensor *ps) {Servo_7_Status = ps;}

// void AnOut_0_Temp(sensor::Sensor *ao) {AnOut_0_Temp_ = ao;}
// void AnOut_1_Vcc(sensor::Sensor *ao) {AnOut_1_Vcc_ = ao;}
void Pump_2_Status(sensor::Sensor *ao) {AnOut_2_LvlPerc_ = ao;}
void Pump_3_Status(sensor::Sensor *ao) {AnOut_3_LvlPerc_ = ao;}
// void Pump_1_Status(sensor::Sensor *ao) {AnOut_0_SensPerc_ = ao;}
// void Pump_1_Status(sensor::Sensor *ao) {AnOut_1_SensPerc_ = ao;}
// void Pump_2_Status(sensor::Sensor *ao) {AnOut_2_SensPerc_ = ao;}
// void Pump_3_Status(sensor::Sensor *ao) {AnOut_3_SensPerc_ = ao;}

// void Pump_1_Status(sensor::Sensor *ps) {DigIn_0_Status_ = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {DigIn_1_Status_ = ps;}
// void Pump_2_Status(sensor::Sensor *ps) {DigIn_2_Status_ = ps;}
// void Pump_3_Status(sensor::Sensor *ps) {DigIn_3_Status_ = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {DigOut_0_Status_ = ps;}
// void Pump_1_Status(sensor::Sensor *ps) {DigOut_1_Status_ = ps;}
// void Pump_2_Status(sensor::Sensor *ps) {DigOut_2_Status_ = ps;}
// void Pump_3_Status(sensor::Sensor *ps) {DigOut_3_Status_ = ps;}


protected:

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
sensor::Sensor *AnOut_0_Temp_{nullptr};
sensor::Sensor *AnOut_1_Vcc_{nullptr};
sensor::Sensor *AnOut_2_LvlPerc_{nullptr};
sensor::Sensor *AnOut_3_LvlPerc_{nullptr};
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
