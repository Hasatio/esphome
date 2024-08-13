#pragma once

#ifndef WQ_I2C_H
#define WQ_I2C_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include "water_quality.h"

namespace esphome {
namespace water_quality {

// I2C Address
#define TCA9548_ADDRESS 0x70
#define ADS1X15_ADDRESS1 0x48
#define ADS1X15_ADDRESS2 0x49
#define MCP23008_ADDRESS 0x20
#define PCA9685_I2C_ADDRESS 0x40

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class WaterQuality;

class WQ_I2C
{
private:
WaterQuality *parent_;
friend class WaterQuality;

public:
// WQ_I2C(WaterQuality *parent);
// WQ_I2C() : i2c::I2CDevice(nullptr) {}
// WQ_I2C(WaterQuality *parent) : parent_(parent) {}
// WQ_I2C(i2c::I2CComponent *parent) : i2c::I2CDevice(parent) {}


// void set_parent(WaterQuality *parent) { parent_ = parent; }

protected:
};

}  // namespace water_quality
}  // namespace esphome

#endif  // WQ_I2C_H