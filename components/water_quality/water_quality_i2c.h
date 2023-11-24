#pragma once

// #ifndef I2C_H
// #define I2C_H

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
// #include "esphome/components/i2c/i2c.h"
// #include <Wire.h>
// #include <Adafruit_ADS1X15.h>
// #include <Adafruit_MCP23X17.h>
// #include <Adafruit_MCP23X08.h>
#include <vector>

    // I2C Address
    #define TCA9548_ADDRESS 0x70
    #define ADS1X15_ADDRESS1 0x48
    #define ADS1X15_ADDRESS2 0x49
    #define MCP23008_ADDRESS 0x20
    #define PCA9685_I2C_ADDRESS 0x40


static const uint8_t ADS1115_REGISTER_CONVERSION = 0x00;
static const uint8_t ADS1115_REGISTER_CONFIG = 0x01;

static const uint8_t ADS1115_DATA_RATE_860_SPS = 0b111;  // 3300_SPS for ADS1015

namespace esphome {
namespace water_quality {

void tcaselect(uint8_t bus);

// void ADS1115_Setup(uint8_t address);
// void ADS1115_Driver(float analog_voltage[]);
void MCP23008_Setup();
void MCP23008_Driver();


// float request_measurement(ADS1115Multiplexer multi);
// void set_continuous_mode(bool continuous_mode) { continuous_mode_ = continuous_mode; }
// void set_multiplexer(ADS1115Multiplexer multiplexer) { multiplexer_ = ADS1115_MULTIPLEXER_P1_NG /*multiplexer*/; }
// void set_gain(ADS1115Gain gain) { gain_ = ADS1115_GAIN_6P144 /*gain*/; }
// void set_resolution(ADS1115Resolution resolution) { resolution_ = ADS1115_16_BITS /*resolution*/; }

// uint8_t get_continuous_mode() const { return continuous_mode_; }
// uint8_t get_multiplexer() const { return multiplexer_; }
// uint8_t get_gain() const { return gain_; }
// uint8_t get_resolution() const { return resolution_; }

}  // namespace water_quality
}  // namespace esphome

// #endif  // I2C_H