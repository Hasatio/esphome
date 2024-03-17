#pragma once

#ifndef WATER_QUALITY_I2C_H
#define WATER_QUALITY_I2C_H

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
#define ADS1X15_ADDRESS1 0x48
#define ADS1X15_ADDRESS2 0x49
#define MCP23008_ADDRESS 0x20
#define PCA9685_I2C_ADDRESS 0x40
#define EZOPMP_I2C_ADDRESS 0x67

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
//  EZOPMP
static const uint16_t EZO_PMP_COMMAND_NONE = 0;
static const uint16_t EZO_PMP_COMMAND_TYPE_READ = 1;

static const uint16_t EZO_PMP_COMMAND_FIND = 2;
static const uint16_t EZO_PMP_COMMAND_DOSE_CONTINUOUSLY = 4;
static const uint16_t EZO_PMP_COMMAND_DOSE_VOLUME = 8;
static const uint16_t EZO_PMP_COMMAND_DOSE_VOLUME_OVER_TIME = 16;
static const uint16_t EZO_PMP_COMMAND_DOSE_WITH_CONSTANT_FLOW_RATE = 32;
static const uint16_t EZO_PMP_COMMAND_SET_CALIBRATION_VOLUME = 64;
static const uint16_t EZO_PMP_COMMAND_CLEAR_TOTAL_VOLUME_DOSED = 128;
static const uint16_t EZO_PMP_COMMAND_CLEAR_CALIBRATION = 256;
static const uint16_t EZO_PMP_COMMAND_PAUSE_DOSING = 512;
static const uint16_t EZO_PMP_COMMAND_STOP_DOSING = 1024;
static const uint16_t EZO_PMP_COMMAND_CHANGE_I2C_ADDRESS = 2048;
static const uint16_t EZO_PMP_COMMAND_EXEC_ARBITRARY_COMMAND_ADDRESS = 4096;

static const uint16_t EZO_PMP_COMMAND_READ_DOSING = 3;
static const uint16_t EZO_PMP_COMMAND_READ_SINGLE_REPORT = 5;
static const uint16_t EZO_PMP_COMMAND_READ_MAX_FLOW_RATE = 9;
static const uint16_t EZO_PMP_COMMAND_READ_PAUSE_STATUS = 17;
static const uint16_t EZO_PMP_COMMAND_READ_TOTAL_VOLUME_DOSED = 33;
static const uint16_t EZO_PMP_COMMAND_READ_ABSOLUTE_TOTAL_VOLUME_DOSED = 65;
static const uint16_t EZO_PMP_COMMAND_READ_CALIBRATION_STATUS = 129;
static const uint16_t EZO_PMP_COMMAND_READ_PUMP_VOLTAGE = 257;

static const std::string DOSING_MODE_NONE = "None";
static const std::string DOSING_MODE_VOLUME = "Volume";
static const std::string DOSING_MODE_VOLUME_OVER_TIME = "Volume/Time";
static const std::string DOSING_MODE_CONSTANT_FLOW_RATE = "Constant Flow Rate";
static const std::string DOSING_MODE_CONTINUOUS = "Continuous";

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

#endif  // WATER_QUALITY_I2C_H