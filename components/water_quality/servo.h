#pragma once

#ifndef SERVO_H
#define SERVO_H

#include "mux.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>

namespace esphome {
namespace water_quality {

class Servo
{
public:

std::vector<bool> Servo_Mode{0,0,0,0,0,0,0,0};
std::vector<uint8_t> Servo_Position{0,0,0,0,0,0,0,0};
std::vector<bool> Servo_Status{0,0,0,0,0,0,0,0};
protected:

};

}  // namespace water_quality
}  // namespace esphome

#endif  // SERVO_H