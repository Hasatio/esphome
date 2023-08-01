#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/time.h"
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MAX1704X.h>
#include <BluetoothSerial.h>

namespace esphome {
namespace myi2c {

class Myi2c : public PollingComponent // ana sınıf
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; } // çalışma önceliği

void setup() override; // ayar fonksiyonu 

void loop() override; // döngü fonksiyonu

void update();

void bluetooth(String b); // bluetooth fonksiyonu

void gain(float g); // kazanç fonksiyonu

#ifdef USE_SENSOR
void my_sample(sensor::Sensor *sample) // sayaç sensörü fonksiyonu
{ 
    sample_ = sample;
}

void my_sample_sec(sensor::Sensor *my_sample_sec) // sayaç sensörü fonksiyonu
{ 
    my_sample_sec_ = my_sample_sec;
}
#endif

void dump_config() override;

protected:

#ifdef USE_SENSOR
sensor::Sensor *my_sample_{nullptr}; // sensör değişkeni
sensor::Sensor *my_sample_sec_{nullptr}; // sensör değişkeni
#endif

}; // class Myi2c
} //namespace myi2c
} //namespace esphome
