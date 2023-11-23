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

namespace esphome {
namespace water_quality {

void tcaselect(uint8_t bus);

void ADS1115_Setup();
void ADS1115_Driver(float analog_voltage[]);
void MCP23008_Setup();
void MCP23008_Driver();

}  // namespace water_quality
}  // namespace esphome

// #endif  // I2C_H