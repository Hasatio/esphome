#pragma once

#ifndef DIGITAL_H
#define DIGITAL_H

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

class Digital
{
public:
void Digital_Input_Driver(bool inputs[]);
void Digital_Output_Driver(bool outputs[]);

void set_Digital_In(bool in[])    {for (size_t i = 0; i < 4; i++) DigIn_Status[i] = in[i];}
void set_Digital_Out(bool out[])  {for (size_t i = 0; i < 4; i++) DigOut_Status[i] = out[i];}

bool* get_Digital_In()  {return DigIn_Status;}
bool* get_Digital_Out() {return DigOut_Status;}

protected:
uint8_t DigIn_FilterCoeff[4] = {0};
bool DigIn_Status[4] = {1,1,1,1};
bool DigOut_Status[4] = {0};

};

}  // namespace water_quality
}  // namespace esphome

#endif  // DIGITAL_H