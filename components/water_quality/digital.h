#pragma once

#include "mux.h"
#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include "Adafruit_MCP23X08.h"

namespace esphome {
namespace water_quality {

static const char *const digital = "digital";

class Digital
{
public:
void mcp23008_set();
void mcp23008();

protected:

};

}  // namespace water_quality
}  // namespace esphome