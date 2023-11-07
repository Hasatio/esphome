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

class Digital
{
public:
static const char *const TAG;

std::vector<std::vector<uint8_t>> DigIn_FilterCoeff{0};
std::vector<bool> DigIn_Read{0,0,0,0};
std::vector<bool> DigIn_Status{1,1,1,1};

void mcp23008_set();
void mcp23008();

protected:

};

}  // namespace water_quality
}  // namespace esphome