#pragma once

#ifndef WATER_QUALITY_I2C_H
#define WATER_QUALITY_I2C_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
// #include <Wire.h>

namespace esphome {
namespace water_quality {

// I2C Address
#define ADS1X15_ADDRESS1 0x48
#define ADS1X15_ADDRESS2 0x49
#define MCP23008_ADDRESS 0x20
#define PCA9685_I2C_ADDRESS 0x40

const uint8_t PCA9685_MODE_INVERTED = 0x10;
const uint8_t PCA9685_MODE_OUTPUT_ONACK = 0x08;
const uint8_t PCA9685_MODE_OUTPUT_TOTEM_POLE = 0x04;
const uint8_t PCA9685_MODE_OUTNE_HIGHZ = 0x02;
const uint8_t PCA9685_MODE_OUTNE_LOW = 0x01;

static const uint8_t PCA9685_REGISTER_SOFTWARE_RESET = 0x06;
static const uint8_t PCA9685_REGISTER_MODE1 = 0x00;
static const uint8_t PCA9685_REGISTER_MODE2 = 0x01;
static const uint8_t PCA9685_REGISTER_LED0 = 0x06;
static const uint8_t PCA9685_REGISTER_PRE_SCALE = 0xFE;

static const uint8_t PCA9685_MODE1_RESTART = 0b10000000;
static const uint8_t PCA9685_MODE1_EXTCLK = 0b01000000;
static const uint8_t PCA9685_MODE1_AUTOINC = 0b00100000;
static const uint8_t PCA9685_MODE1_SLEEP = 0b00010000;

class WaterQuality;

class I2C : public i2c::I2CDevice
{
public:
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

void test();

void set_water_quality(WaterQuality *wq)
{
    wq_ = wq;
}

private:
friend class WaterQuality;

protected:
uint16_t prev_config_{0};
ADS1115_Multiplexer multiplexer_;
ADS1115_Gain gain_;
bool continuous_mode_;
ADS1115_DataRate data_rate_;
ADS1115_Resolution resolution_;

WaterQuality *wq_;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WATER_QUALITY_I2C_H