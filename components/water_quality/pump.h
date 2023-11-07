#pragma once

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

static const char *const pump = "pump";

uint16_t PwmFreq = 1000;

class Pump
{
public:
void pump_total();

protected:

};

}  // namespace water_quality
}  // namespace esphome