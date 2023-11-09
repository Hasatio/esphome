#pragma once

#ifndef PUMP_H
#define PUMP_H

#include "../water_quality.h"
#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

class Pump
{
public:

void pump_total();
void sensor();
};

}  // namespace water_quality
}  // namespace esphome

#endif  // PUMP_H