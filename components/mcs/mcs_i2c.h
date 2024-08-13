#pragma once

#ifndef MCS_I2C_H
#define MCS_I2C_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include "mcs.h"

namespace esphome {
namespace mcs {

// I2C Address
#define TCA9548_ADDRESS 0x70
#define MCP23017_ADDRESS1 0x20
#define MCP23017_ADDRESS2 0x21

class MCS;

class MCS_I2C
{
private:
MCS *parent_;
friend class MCS;

public:

protected:
};

}  // namespace mcs
}  // namespace esphome

#endif  // MCS_I2C_H