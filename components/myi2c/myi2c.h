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

void bluetooth(char *name);

void gain(float g);
    
void setup() override;

void loop() override;

};
} //namespace myi2c
} //namespace esphome
