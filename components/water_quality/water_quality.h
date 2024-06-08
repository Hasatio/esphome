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
#include <cmath>
#include <EEPROM.h>
#include "wq_i2c.h"

#define EEPROM_SIZE 64 // 64 byte
#define PH1ADDR 0x00
#define Volt1ADDR 0x04
#define PH2ADDR 0x08
#define Volt2ADDR 0x0c

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

void version(const uint8_t ver);
void pump_calibration_mode(std::vector<bool> &pres);
void pump_calibration_gain(const std::vector<float> &pcal);
void pump_type(const std::vector<uint8_t> &ptype);
void pump_mode(std::vector<uint8_t> &pmode);
void pump_dose(std::vector<float> &pdose);
void pump_circulation(std::vector<float> &pcirc);
void pump_reset(std::vector<bool> &pres);
void servo_mode(std::vector<bool> &smode);
void servo_position(std::vector<uint8_t> &spos);
void level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax);
void ph_calibration(float cal);
void ph(const uint8_t ch, const uint8_t type);
void ec_calibration(float cal);
void ec(const uint8_t ch, const uint8_t type);
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
template<typename... Ts> class Pump_Calibration_Mode_Action : public Action<Ts...> {
public:
Pump_Calibration_Mode_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, pump_calib_mode);

void play(Ts... x) 
{
    std::vector<bool> data = this->pump_calib_mode_.value(x...);

    this->parent_->pump_calibration_mode(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class Pump_Mode_Action : public Action<Ts...> {
public:
Pump_Mode_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<uint8_t>, pump_m);

void play(Ts... x) 
{
    std::vector<uint8_t> data = this->pump_m_.value(x...);

    this->parent_->pump_mode(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class Pump_Dose_Action : public Action<Ts...> {
public:
Pump_Dose_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<float>, pump_d);

void play(Ts... x) 
{
    std::vector<float> data = this->pump_d_.value(x...);

    this->parent_->pump_dose(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class Pump_Circulation_Action : public Action<Ts...> {
public:
Pump_Circulation_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<float>, pump_c);

void play(Ts... x) 
{
    std::vector<float> data = this->pump_c_.value(x...);

    this->parent_->pump_circulation(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class Pump_Reset_Action : public Action<Ts...> {
public:
Pump_Reset_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, pump_res);

void play(Ts... x) 
{
    std::vector<bool> data = this->pump_res_.value(x...);

    this->parent_->pump_reset(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class Servo_Mode_Action : public Action<Ts...> {
public:
Servo_Mode_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, ser_mode);

void play(Ts... x) 
{
    std::vector<bool> data = this->ser_mode_.value(x...);

    this->parent_->servo_mode(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class Servo_Position_Action : public Action<Ts...> {
public:
Servo_Position_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<uint8_t>, ser_pos);

void play(Ts... x) 
{
    std::vector<uint8_t> data = this->ser_pos_.value(x...);

    this->parent_->servo_position(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class PH_Calibration_Action : public Action<Ts...> {
public:
PH_Calibration_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(float, ph_cal);

void play(Ts... x) 
{
    float data = this->ph_cal_.value(x...);

    this->parent_->ph_calibration(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class EC_Calibration_Action : public Action<Ts...> {
public:
EC_Calibration_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(float, ec_cal);

void play(Ts... x) 
{
    float data = this->ec_cal_.value(x...);

    this->parent_->ec_calibration(data);
}

protected:
WaterQuality *parent_;
};
template<typename... Ts> class Digital_Out_Action : public Action<Ts...> {
public:
Digital_Out_Action(WaterQuality *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, dig_out);

void play(Ts... x) 
{
    std::vector<bool> data = this->dig_out_.value(x...);

    this->parent_->digital_out(data);
}

protected:
WaterQuality *parent_;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WATER_QUALITY_H