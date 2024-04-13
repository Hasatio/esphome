#pragma once

#ifndef WATER_QUALITY_H
#define WATER_QUALITY_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <Wire.h>
#include "wq_i2c.h"

namespace esphome {
namespace water_quality {

static const char *const TAG = "mycomponent";

enum ADS1115_Registers
{
    ADS1115_REGISTER_CONVERSION = 0x00,
    ADS1115_REGISTER_CONFIG = 0x01,
};
enum ADS1115_Multiplexer
{
    ADS1115_MULTIPLEXER_P0_N1 = 0b000,
    ADS1115_MULTIPLEXER_P0_N3 = 0b001,
    ADS1115_MULTIPLEXER_P1_N3 = 0b010,
    ADS1115_MULTIPLEXER_P2_N3 = 0b011,
    ADS1115_MULTIPLEXER_P0_NG = 0b100,
    ADS1115_MULTIPLEXER_P1_NG = 0b101,
    ADS1115_MULTIPLEXER_P2_NG = 0b110,
    ADS1115_MULTIPLEXER_P3_NG = 0b111,
};
enum ADS1115_Gain
{
    ADS1115_GAIN_6P144 = 0b000,
    ADS1115_GAIN_4P096 = 0b001,
    ADS1115_GAIN_2P048 = 0b010,
    ADS1115_GAIN_1P024 = 0b011,
    ADS1115_GAIN_0P512 = 0b100,
    ADS1115_GAIN_0P256 = 0b101,
};
enum ADS1115_DataRate
{
    ADS1115_DATA_RATE_8_SPS = 0b000,
    ADS1115_DATA_RATE_16_SPS = 0b001,
    ADS1115_DATA_RATE_32_SPS = 0b010,
    ADS1115_DATA_RATE_64_SPS = 0b011,
    ADS1115_DATA_RATE_128_SPS = 0b100,
    ADS1115_DATA_RATE_250_SPS = 0b101,
    ADS1115_DATA_RATE_475_SPS = 0b110,
    ADS1115_DATA_RATE_860_SPS = 0b111,
};
enum ADS1115_Resolution
{
    ADS1115_16_BITS = 16,
    ADS1015_12_BITS = 12,
};

enum MCP23008_Registers 
{
    MCP23008_IODIR = 0x00,
    MCP23008_IPOL = 0x01,
    MCP23008_GPINTEN = 0x02,
    MCP23008_DEFVAL = 0x03,
    MCP23008_INTCON = 0x04,
    MCP23008_IOCON = 0x05,
    MCP23008_GPPU = 0x06,
    MCP23008_INTF = 0x07,
    MCP23008_INTCAP = 0x08,
    MCP23008_GPIO = 0x09,
    MCP23008_OLAT = 0x0A,
};
enum MCP23008_PinMode
{
    FLAG_NONE,
    FLAG_INPUT,
    FLAG_OUTPUT,
    FLAG_OPEN_DRAIN,
    FLAG_PULLUP,
    FLAG_PULLDOWN,
};
enum MCP23008_InterruptMode : uint8_t
{
    MCP23008_NO_INTERRUPT = 0,
    MCP23008_CHANGE,
    MCP23008_RISING,
    MCP23008_FALLING,
};

class WQ_I2C;

class WaterQuality : public PollingComponent, public i2c::I2CDevice
{
private:
// WQ_I2C *wq_i2c_;
// friend class WQ_I2C;
// friend class Pump;

public:
// WaterQuality();
// WaterQuality(i2c::I2CComponent *parent) : I2C(parent) {}
// WaterQuality(WQ_I2C *i2c) : wq_i2c_(i2c) {}

float get_setup_priority() const override { return esphome::setup_priority::DATA; }

void setup() override;
void dump_config() override;
void loop() override;
void update() override;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548
void tcaselect(uint8_t i)
{
  if (i > 7) return;
 
  Wire.beginTransmission(TCA9548_ADDRESS);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
void ADS1115_Setup(uint8_t address);
float ADS1115_Read();
void ADS1115_Driver(float analog_voltage[]);

void set_multiplexer(ADS1115_Multiplexer multiplexer)   { multiplexer_ = multiplexer; }
void set_gain(ADS1115_Gain gain)                        { gain_ = gain; }
void set_continuous_mode(bool continuous_mode)          { continuous_mode_ = continuous_mode; }
void set_data_rate(ADS1115_DataRate data_rate)          { data_rate_ = data_rate; }
void set_resolution(ADS1115_Resolution resolution)      { resolution_ = resolution; }

uint8_t get_multiplexer() const     { return multiplexer_; }
uint8_t get_gain() const            { return gain_; }
bool get_continuous_mode()          { return continuous_mode_; }
uint8_t get_data_rate() const       { return data_rate_; }
uint8_t get_resolution() const      { return resolution_; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008
void MCP23008_Setup(uint8_t address);
uint8_t MCP23008_Read();
void MCP23008_Write(bool value[]);
void MCP23008_Driver(bool digital[]);
void MCP23008_pin_interrupt_mode(uint8_t pin, MCP23008_InterruptMode interrupt_mode);

void set_open_drain_ints(const bool value)                      { this->open_drain_ints_ = value; }
void set_pin(uint8_t pin)                                       { pin_ = pin; }
void set_inverted(bool inverted)                                { inverted_ = inverted; }
void set_interrupt_mode(MCP23008_InterruptMode interrupt_mode)  { interrupt_mode_ = interrupt_mode; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685
void PCA9685_Setup(uint8_t address);
void PCA9685_Write();
void PCA9685_Driver(float state[]);

void set_extclk(bool extclk)            { this->extclk_ = extclk; }
void set_frequency(float frequency)     { this->frequency_ = frequency; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  EZOPMP
// void EZOPMP_Read();
// void EZOPMP_Write();
// void EZOPMP_Driver(float volume[]);
// void EZOPMP_loop();
// void EZOPMP_update();

// void find();
// void dose_continuously();
// void dose_volume(float volume);
// void dose_volume_over_time(float volume, int duration);
// void dose_with_constant_flow_rate(float volume, int duration);
// void set_calibration_volume(float volume);
// void clear_total_volume_dosed();
// void clear_calibration();
// void pause_dosing();
// void stop_dosing();
// void change_i2c_address(int address);
// void exec_arbitrary_command(const std::basic_string<char> &command);
// void custom_command(std::string custom);

// void set_current_volume_dosed(float current_volume_dosed)                   { current_volume_dosed_ = current_volume_dosed; }
// void set_total_volume_dosed(float total_volume_dosed)                       { total_volume_dosed_ = total_volume_dosed; }
// void set_absolute_total_volume_dosed(float absolute_total_volume_dosed)     { absolute_total_volume_dosed_ = absolute_total_volume_dosed; }
// void set_pump_voltage(float pump_voltage)                                   { pump_voltage_ = pump_voltage; }
// void set_last_volume_requested(float last_volume_requested)                 { last_volume_requested_ = last_volume_requested; }
// void set_max_flow_rate(float max_flow_rate)                                 { max_flow_rate_ = max_flow_rate; }
// void set_is_dosing(bool is_dosing)                                          { is_dosing_ = is_dosing; }
// void set_is_paused(bool is_paused)                                          { is_paused_ = is_paused; }
// void set_dosing_mode(std::string dosing_mode)                               { dosing_mode_ = dosing_mode; }
// void set_calibration_status(std::string calibration_status)                 { calibration_status_ = calibration_status; }

// float get_current_volume_dosed()            { return current_volume_dosed_; }
// float get_total_volume_dosed()              { return total_volume_dosed_; }
// float get_absolute_total_volume_dosed()     { return absolute_total_volume_dosed_; }
// float get_pump_voltage()                    { return pump_voltage_; }
// float get_last_volume_requested()           { return last_volume_requested_; }
// float get_max_flow_rate()                   { return max_flow_rate_; }
// bool get_is_dosing()                        { return is_dosing_; }
// bool get_is_paused()                        { return is_paused_; }
// std::string get_dosing_mode()               { return dosing_mode_; }
// std::string get_calibration_status()        { return calibration_status_; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void version(const uint8_t ver);
void pump_calibration_mode(std::vector<bool> &pres);
void pump_calibration_gain(const std::vector<float> &pcal);
void pump_type(const std::vector<uint8_t> &ptype);
void pump_model(const std::vector<uint8_t> &pmodel);
void pump_mode(std::vector<uint8_t> &pmode);
void pump_dose(std::vector<float> &pdose);
void pump_circulation(std::vector<float> &pcirc);
void pump_reset(std::vector<bool> &pres);
void servo_mode(std::vector<bool> &smode);
void servo_position(std::vector<uint8_t> &spos);
void level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax);
void ec(const uint8_t ch, const uint8_t type);
void ph(const uint8_t ch, const uint8_t type);
void digital_out(std::vector<bool> &dout);

void sensor();

void Pump_Tot_Sensor    (text_sensor::TextSensor *ptot)     { Pump_Tot_ = ptot; }
void Pump_Stat_Sensor   (text_sensor::TextSensor *pstat)    { Pump_Stat_ = pstat; }
void Servo_Stat_Sensor  (text_sensor::TextSensor *servo)    { Servo_Stat_ = servo; }
void WTemp_Val_Sensor   (sensor::Sensor *wtemp)             { AnInWTemp_Val_ = wtemp; }
void VPow_Val_Sensor    (sensor::Sensor *vpow)              { AnInVPow_Val_ = vpow; }
void AnLvl_Perc_Sensor  (text_sensor::TextSensor *level)    { AnInLvl_Perc_ = level; }
void EC_Val_Sensor      (sensor::Sensor *ec)                { AnInEC_Val_ = ec; }
void PH_Val_Sensor      (sensor::Sensor *ph)                { AnInPH_Val_ = ph; }
void AnGen_Val_Sensor   (text_sensor::TextSensor *gen)      { AnInGen_Val_ = gen; }
void DigIn_Stat_Sensor  (text_sensor::TextSensor *din)      { DigIn_Stat_ = din; }

protected:
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
uint16_t prev_config_{0};
ADS1115_Multiplexer multiplexer_;
ADS1115_Gain gain_;
bool continuous_mode_;
ADS1115_DataRate data_rate_;
ADS1115_Resolution resolution_;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008
uint8_t olat_{0x00};
uint8_t pin_;
bool inverted_;
MCP23008_InterruptMode interrupt_mode_;
bool open_drain_ints_;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685
float frequency_ = 1000;
bool extclk_ = false;
uint16_t pwm_amounts_[16] = {0};
bool update_{true};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  EZOPMP
// uint32_t start_time_ = 0;
// uint32_t wait_time_ = 0;
// bool is_waiting_ = false;
// bool is_first_read_ = true;

// uint16_t next_command_ = 0;
// double next_command_volume_ = 0;  // might be negative
// int next_command_duration_ = 0;

// uint16_t next_command_queue_[10];
// double next_command_volume_queue_[10];
// int next_command_duration_queue_[10];
// int next_command_queue_head_ = 0;
// int next_command_queue_last_ = 0;
// int next_command_queue_length_ = 0;

// uint16_t current_command_ = 0;

// const char *arbitrary_command_{nullptr};

// void send_next_command_();
// void read_command_result_();
// void clear_current_command_();
// void queue_command_(uint16_t command, float volume, int duration, bool should_schedule);
// void pop_next_command_();
// uint16_t peek_next_command_();

// uint16_t command_[50] = {0};
// uint16_t command2_[50] = {0};
// float volume_[6] = {0};
// float current_volume_dosed_ = 0;
// float total_volume_dosed_ = 0;
// float absolute_total_volume_dosed_ = 0;
// float pump_voltage_ = 0;
// float max_flow_rate_ = 0;
// float last_volume_requested_ = 0;
// bool is_dosing_ = 0;
// bool is_paused_ = 0;
// std::string custom_ = {0};
// std::string dosing_mode_ = {0};
// std::string calibration_status_ = {0};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor
text_sensor::TextSensor *Pump_Tot_{nullptr};
text_sensor::TextSensor *Pump_Stat_{nullptr};
text_sensor::TextSensor *Servo_Stat_{nullptr};
sensor::Sensor *AnInWTemp_Val_{nullptr};
sensor::Sensor *AnInVPow_Val_{nullptr};
text_sensor::TextSensor *AnInLvl_Perc_{nullptr};
sensor::Sensor *AnInEC_Val_{nullptr};
sensor::Sensor *AnInPH_Val_{nullptr};
text_sensor::TextSensor *AnInGen_Val_{nullptr};
text_sensor::TextSensor *DigIn_Stat_{nullptr};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};
template<typename... Ts> class PumpCalibrationModeAction : public Action<Ts...> {
public:
PumpCalibrationModeAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, pump_calib_mode);

void play(Ts... x) 
{
    std::vector<bool> data = this->pump_calib_mode_.value(x...);

    this->parent_->pump_calibration_mode(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class PumpModeAction : public Action<Ts...> {
public:
PumpModeAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<uint8_t>, pump_m);

void play(Ts... x) 
{
    std::vector<uint8_t> data = this->pump_m_.value(x...);

    this->parent_->pump_mode(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class PumpDoseAction : public Action<Ts...> {
public:
PumpDoseAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<float>, pump_d);

void play(Ts... x) 
{
    std::vector<float> data = this->pump_d_.value(x...);

    this->parent_->pump_dose(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class PumpCirculationAction : public Action<Ts...> {
public:
PumpCirculationAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<float>, pump_c);

void play(Ts... x) 
{
    std::vector<float> data = this->pump_c_.value(x...);

    this->parent_->pump_circulation(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class PumpResetAction : public Action<Ts...> {
public:
PumpResetAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, pump_res);

void play(Ts... x) 
{
    std::vector<bool> data = this->pump_res_.value(x...);

    this->parent_->pump_reset(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class ServoModeAction : public Action<Ts...> {
public:
ServoModeAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, ser_mode);

void play(Ts... x) 
{
    std::vector<bool> data = this->ser_mode_.value(x...);

    this->parent_->servo_mode(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class ServoPositionAction : public Action<Ts...> {
public:
ServoPositionAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<uint8_t>, ser_pos);

void play(Ts... x) 
{
    std::vector<uint8_t> data = this->ser_pos_.value(x...);

    this->parent_->servo_position(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class DigitalOutAction : public Action<Ts...> {
public:
DigitalOutAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, dig_out);

void play(Ts... x) 
{
    std::vector<bool> data = this->dig_out_.value(x...);

    this->parent_->digital_out(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class CustomCommandAction : public Action<Ts...> {
public:
CustomCommandAction(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::string, cus_com);

void play(Ts... x) 
{
    std::string data = this->cus_com_.value(x...);

    this->parent_->custom_command(data);
}

protected:
WaterQuality *parent_;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WATER_QUALITY_H