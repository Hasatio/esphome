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
void Digital_Input_Driver(bool input[]);
void Digital_Output_Driver(bool output[]);

void set_Digital_Timepoint(uint32_t time)       { DigIn_Timepoint = time; }
void set_Digital_FilterCoeff(uint8_t filter[])  { for (size_t i = 0; i < 4; i++)  DigIn_FilterCoeff[i] = filter[i]; }
void set_Digital_Input(bool in[])               { for (size_t i = 0; i < 4; i++)  DigIn_Status[i] = in[i]; }
void set_Digital_Output(bool out[])             { for (size_t i = 0; i < 4; i++)  DigOut_Status[i] = out[i]; }

uint32_t get_Digital_Timepoint()    { return DigIn_Timepoint; }
uint8_t* get_Digital_FilterCoeff()  { return DigIn_FilterCoeff; }
bool* get_Digital_In()              { return DigIn_Status; }
bool* get_Digital_Out()             { return DigOut_Status; }

protected:
uint32_t DigIn_Timepoint = 0;
uint8_t DigIn_FilterCoeff[4] = {0};
bool DigIn_Status[4] = {1,1,1,1};
bool DigOut_Status[4] = {0};

};

}  // namespace water_quality
}  // namespace esphome

#endif  // DIGITAL_H