#pragma once

#ifndef DIGITAL_H
#define DIGITAL_H

#include "i2c.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>

namespace esphome {
namespace water_quality {

class Digital
{
public:
std::vector<std::vector<uint8_t>> DigIn_FilterCoeff{0};
std::vector<bool> DigIn_Read{0,0,0,0};
std::vector<bool> DigIn_Status{1,1,1,1};
std::vector<bool> DigOut_Status{0,0,0,0};
protected:

};

}  // namespace water_quality
}  // namespace esphome

#endif  // DIGITAL_H