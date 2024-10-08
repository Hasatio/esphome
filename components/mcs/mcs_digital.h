#pragma once

#ifndef MCS_DIGITAL_H
#define MCS_DIGITAL_H

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mcs {

class MCS_Digital
{
public:
void Digital_Output_Driver(bool output[]);

void set_Digital_Output(bool out[])     { for (uint8_t i = 0; i < 20; i++)  DigOut_Status[i] = out[i]; }
void set_Digital_Output2(uint8_t out)   { DigOut = out; }

bool* get_Digital_Output()      { return DigOut_Status; }
uint8_t get_Digital_Output2()   { return DigOut; }

protected:
bool DigOut_Status[20] = {0};
uint8_t DigOut = 0;

};

}  // namespace mcs
}  // namespace esphome

#endif  // MCS_DIGITAL_H