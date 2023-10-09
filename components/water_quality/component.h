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
};

void Pump_0_Total(sensor::Sensor *pt) Pump_0_Total_ = pt;
void Pump_1_Total(sensor::Sensor *pt) Pump_1_Total_ = pt;
void Pump_2_Total(sensor::Sensor *pt) Pump_2_Total_ = pt;
void Pump_3_Total(sensor::Sensor *pt) Pump_3_Total_ = pt;
void Pump_0_Status(sensor::Sensor *ps) Pump_0_Status_ = ps;
void Pump_1_Status(sensor::Sensor *ps) Pump_1_Status_ = ps;   
void Pump_2_Status(sensor::Sensor *ps) Pump_2_Status_ = ps; 
void Pump_3_Status(sensor::Sensor *ps) Pump_3_Status_ = ps;


protected:

sensor::Sensor *Pump_0_Total_{nullptr};
sensor::Sensor *Pump_1_Total_{nullptr};
sensor::Sensor *Pump_2_Total_{nullptr};
sensor::Sensor *Pump_3_Total_{nullptr};
sensor::Sensor *Pump_0_Status_{nullptr};
sensor::Sensor *Pump_1_Status_{nullptr};
sensor::Sensor *Pump_2_Status_{nullptr};
sensor::Sensor *Pump_3_Status_{nullptr};
 
}  // namespace water_quality
}  // namespace esphome
