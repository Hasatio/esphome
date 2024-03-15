#pragma once

#ifndef PUMP_H
#define PUMP_H

#include "esphome.h"
#include "esphome/core/log.h"
#include <cmath>
#include <thread>
#include <esp_timer.h>

namespace esphome {
namespace water_quality {

class Pump
{
public:
void Timer_Setup(float period);
static void IRAM_ATTR Timer(void* arg);
void Calibration_Status();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GENERIC
void Generic_Pump_Driver(float pwm[]);
void Dosing_Controller(float pump[]);
void Circulation_Controller(float pump[]);

void set_Calibration_Mode(bool cm)      { Calibration_Mode = cm; }
void set_Pump_Calib_Gain(float pcg[])   { for (size_t i = 0; i < 6; i++) Pump_Calib_Gain[i] = pcg[i]; }
void set_Pump_Type(uint8_t pt[])        { for (size_t i = 0; i < 6; i++) Pump_Type[i] = pt[i]; }
void set_Pump_Model(uint8_t pm[])       { for (size_t i = 0; i < 6; i++) Pump_Model[i] = pm[i]; }
void set_Pump_Mode(uint8_t pm[])        { for (size_t i = 0; i < 6; i++) Pump_Mode[i] = pm[i]; }
void set_Pump_Status(uint8_t ps[])      { for (size_t i = 0; i < 6; i++) Pump_Status[i] = ps[i]; }
void set_Pump_Dose(float pd[])          { for (size_t i = 0; i < 6; i++) Pump_Dose[i] = pd[i]; }
void set_Pump_Circulation(float pc[])   { for (size_t i = 0; i < 6; i++) Pump_Circulation[i] = pc[i]; }
void set_Pump_Total(uint32_t pt[][2])   { for (size_t i = 0; i < 6; i++) for (size_t j = 0; j < 2; j++) Pump_Total[i][j] = pt[i][j]; }
void set_Pump_Reset(bool pr[])          { for (size_t i = 0; i < 6; i++) Pump_Reset[i] = pr[i]; }
void set_Pump_Time(float pt[])          { for (size_t i = 0; i < 6; i++) Pump_Time[i] = pt[i]; }
void set_Min(float m)                   { min = m; }

bool get_Calibration_Mode()         { return Calibration_Mode; }
float* get_Pump_Calib_Gain()        { return Pump_Calib_Gain; }
uint8_t* get_Pump_Type()            { return Pump_Type; }
uint8_t* get_Pump_Model()           { return Pump_Model; }
uint8_t* get_Pump_Mode()            { return Pump_Mode; }
uint8_t* get_Pump_Status()          { return Pump_Status; }
float* get_Pump_Dose()              { return Pump_Dose; }
float* get_Pump_Circulation()       { return Pump_Circulation; }
uint32_t (*get_Pump_Total())[2]     { return Pump_Total; }
bool* get_Pump_Reset()              { return Pump_Reset; }
float* get_Pump_Time()              { return Pump_Time; }
float get_Min()                     { return min; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SERIAL
double Serial_Com_Pump_Driver();

void find();
void dose_continuously();
void dose_volume(double volume);
void dose_volume_over_time(double volume, int duration);
void dose_with_constant_flow_rate(double volume, int duration);
void set_calibration_volume(double volume);
void clear_total_volume_dosed();
void clear_calibration();
void pause_dosing();
void stop_dosing();
void change_i2c_address(int address);
void exec_arbitrary_command(const std::basic_string<char> &command);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

protected:
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GENERIC
bool Calibration_Mode = 0;
float Pump_Calib_Gain[6] = {0};
uint8_t Pump_Type[6] = {0};
uint8_t Pump_Model[6] = {0};
uint8_t Pump_Mode[6] = {0};
uint8_t Pump_Status[6] = {0};
float Pump_Dose[6] = {0};
float Pump_Circulation[6] = {0};
uint32_t Pump_Total[6][2] = {0};
bool Pump_Reset[6] = {0};
float Pump_Time[6] = {0};
float min = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SERIAL
uint32_t start_time_ = 0;
uint32_t wait_time_ = 0;
bool is_waiting_ = false;
bool is_first_read_ = true;

uint16_t next_command_ = 0;
double next_command_volume_ = 0;  // might be negative
int next_command_duration_ = 0;

uint16_t next_command_queue_[10];
double next_command_volume_queue_[10];
int next_command_duration_queue_[10];
int next_command_queue_head_ = 0;
int next_command_queue_last_ = 0;
int next_command_queue_length_ = 0;

uint16_t current_command_ = 0;
bool is_paused_flag_ = false;
bool is_dosing_flag_ = false;

const char *arbitrary_command_{nullptr};

void clear_current_command_();
void queue_command_(uint16_t command, double volume, int duration, bool should_schedule);
void pop_next_command_();
uint16_t peek_next_command_();

double volume_ = 0;
float current_volume_dosed_= 0;
float total_volume_dosed_= 0;
float absolute_total_volume_dosed_= 0;
float pump_voltage_= 0;
float max_flow_rate_= 0;
float last_volume_requested_= 0;
bool is_dosing_= 0;
bool is_paused_= 0;
std::string dosing_mode_= {0};
std::string calibration_status_= {0};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
esp_timer_handle_t timer;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // PUMP_H