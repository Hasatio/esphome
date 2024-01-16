#pragma once

#ifndef WATER_QUALITY_I2C_H
#define WATER_QUALITY_I2C_H

#include "water_quality.h"
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
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

class Data
{
public:
void test();

void set_water_quality(WaterQuality *wq)
{
    wq_ = wq;
}

private:
friend class WaterQuality;

protected:
WaterQuality *wq_;
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WATER_QUALITY_I2C_H