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

class Myi2c : public sensor::Sensor, public Component // ana sınıf
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; } // çalışma önceliği

void setup() override; // ayar fonksiyonu 

void loop(); // döngü fonksiyonu

void update(); 

void bluetooth(String b); // bluetooth fonksiyonu

void gain(float g); // kazanç fonksiyonu

void sample(sensor::Sensor *sample) // sayaç sensörü fonksiyonu
{ 
    sample_ = sample;
}

void sample_sec(sensor::Sensor *sample_sec) // sayaç sensörü fonksiyonu
{ 
    sample_sec_ = sample_sec;
}

void dump_config() override;

std::string unique_id() override;

protected:

sensor::Sensor *sample_{nullptr}; // sensör değişkeni
sensor::Sensor *sample_sec_{nullptr}; // sensör değişkeni
uint64_t uptime_{0};

}; // class Myi2c
} //namespace myi2c
} //namespace esphome
