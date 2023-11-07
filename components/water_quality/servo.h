#pragma once

#include "mux.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>

namespace esphome {
namespace water_quality {

static const char *const servo = "servo";

std::vector<bool> Servo_Mode{0,0,0,0,0,0,0,0};
std::vector<uint8_t> Servo_Position{0,0,0,0,0,0,0,0};
std::vector<bool> Servo_Status{0,0,0,0,0,0,0,0};

class Servo
{
public:

protected:

};

}  // namespace water_quality
}  // namespace esphome