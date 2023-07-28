#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/time.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MAX1704X.h>
#include <BluetoothSerial.h>

namespace esphome {
namespace myi2c {

class Myi2c : public Component
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void bluetooth(String b);

void gain(float g);

void sample(sensor::Sensor *temperature_sensor) 
{ 
    temperature_sensor_ = temperature_sensor;
    ESP_LOGCONFIG(TAG, "sample:");
    LOG_UPDATE_INTERVAL(this);
    LOG_SENSOR("  ", "sample/sec", this->temperature_sensor_);
}

void setup() override;

void loop() override;

protected:

sensor::Sensor *temperature_sensor_{nullptr};

};
} //namespace myi2c
} //namespace esphome
