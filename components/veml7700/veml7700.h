#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include <Adafruit_VEML7700.h>

// I2C Address
#define VEML7700_I2CADDR_DEFAULT 0x10

#define VEML_GAIN VEML7700_GAIN_1 // GAIN_1 is 'default'
#define VEML_IT VEML7700_IT_100MS // 100MS is 'default'
// gain 1, IT 100mS, means the max light level reported will be 3775

static const char *const TAG = "mycomponent";

namespace esphome {
namespace veml7700 {

class VEML7700 : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor
{
public:

float get_setup_priority() const override { return esphome::setup_priority::DATA; }

void setup() override;
void dump_config() override;
void loop() override;
void update() override;

void Lux      (sensor::Sensor *l)    { Lux_ = l; }
void White    (sensor::Sensor *w)    { White_ = w; }
void Als      (sensor::Sensor *a)    { Als_ = a; }

bool VEML7700Present = false, VEML7700FirstRead = true;

protected:
sensor::Sensor *Lux_ = {nullptr};
sensor::Sensor *White_ = {nullptr};
sensor::Sensor *Als_ = {nullptr};
// Sensor *veml7700_gain = new Sensor();
// Sensor *veml7700_it = new Sensor();
// Sensor *veml7700_interrupt_status = new Sensor();

};

}  // namespace veml7700
}  // namespace esphome