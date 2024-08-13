#pragma once

#ifndef WQ_DIGITAL_H
#define WQ_DIGITAL_H

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

class Digital
{
public:
void Digital_Input_Driver(bool input[]);
void Digital_Output_Driver(bool output[]);

void set_Digital_Timepoint(unsigned long time)  { DigIn_Timepoint = time; }
void set_Digital_FilterCoeff(uint8_t filter[])  { for (uint8_t i = 0; i < 4; i++)  DigIn_FilterCoeff[i] = filter[i]; }
void set_Digital_Input(bool in[])               { for (uint8_t i = 0; i < 4; i++)  DigIn_Status[i] = in[i]; }
void set_Digital_Output(bool out[])             { for (uint8_t i = 0; i < 4; i++)  DigOut_Status[i] = out[i]; }

unsigned long get_Digital_Timepoint()   { return DigIn_Timepoint; }
uint8_t* get_Digital_FilterCoeff()      { return DigIn_FilterCoeff; }
bool* get_Digital_Input()               { return DigIn_Status; }
bool* get_Digital_Output()              { return DigOut_Status; }

protected:
unsigned long DigIn_Timepoint = 0;
uint8_t DigIn_FilterCoeff[4] = {0};
bool DigIn_Status[4] = {1,1,1,1};
bool DigOut_Status[4] = {0};

};

}  // namespace water_quality
}  // namespace esphome

#endif  // WQ_DIGITAL_H