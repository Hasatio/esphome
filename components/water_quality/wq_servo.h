#pragma once

#ifndef SERVO_H
#define SERVO_H

#include "esphome.h"
#include "esphome/core/log.h"

namespace esphome {
namespace water_quality {

class Servo
{
public:
void Servo_driver(float pwm[]);
void Servo_Controller(float perc[]);

void set_Servo_Mode(bool sm[])          {for (size_t i = 0; i < 8; i++) Servo_Mode[i] = sm[i];}
void set_Servo_Position(uint8_t sp[])   {for (size_t i = 0; i < 8; i++) Servo_Position[i] = sp[i];}
void set_Servo_Status(bool ss[])        {for (size_t i = 0; i < 8; i++) Servo_Status[i] = ss[i];}

bool* get_Servo_Mode()          {return Servo_Mode;}
uint8_t* get_Servo_Position()   {return Servo_Position;}
bool* get_Servo_Status()        {return Servo_Status;}

protected:
bool Servo_Mode[8] = {0};
uint8_t Servo_Position[8] = {0};
bool Servo_Status[8] = {0};
};

}  // namespace water_quality
}  // namespace esphome

#endif  // SERVO_H