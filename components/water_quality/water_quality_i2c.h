#pragma once

// #ifndef I2C_H
// #define I2C_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_MCP23X08.h>
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


enum ADS1115Multiplexer {
  ADS1115_MULTIPLEXER_P0_N1 = 0b000,
  ADS1115_MULTIPLEXER_P0_N3 = 0b001,
  ADS1115_MULTIPLEXER_P1_N3 = 0b010,
  ADS1115_MULTIPLEXER_P2_N3 = 0b011,
  ADS1115_MULTIPLEXER_P0_NG = 0b100,
  ADS1115_MULTIPLEXER_P1_NG = 0b101,
  ADS1115_MULTIPLEXER_P2_NG = 0b110,
  ADS1115_MULTIPLEXER_P3_NG = 0b111,
};

enum ADS1115Gain {
  ADS1115_GAIN_6P144 = 0b000,
  ADS1115_GAIN_4P096 = 0b001,
  ADS1115_GAIN_2P048 = 0b010,
  ADS1115_GAIN_1P024 = 0b011,
  ADS1115_GAIN_0P512 = 0b100,
  ADS1115_GAIN_0P256 = 0b101,
};

enum ADS1115Resolution {
  ADS1115_16_BITS = 16,
  ADS1015_12_BITS = 12,
};

class ADS1115Sensor;

class ADS1115Component
{
public:
void register_sensor(ADS1115Sensor *obj) { this->sensors_.push_back(obj); }
/// Set up the internal sensor array.
void setup() override;
void dump_config() override;
/// HARDWARE_LATE setup priority
float get_setup_priority() const override { return setup_priority::DATA; }
void set_continuous_mode(bool continuous_mode) { continuous_mode_ = continuous_mode; }

/// Helper method to request a measurement from a sensor.
float request_measurement(ADS1115Sensor *sensor);

protected:
std::vector<ADS1115Sensor *> sensors_;
uint16_t prev_config_{0};
bool continuous_mode_;
};

/// Internal holder class that is in instance of Sensor so that the hub can create individual sensors.
class ADS1115Sensor
{
public:
ADS1115Sensor(ADS1115Component *parent) : parent_(parent) {}
void update() override;
void set_multiplexer(ADS1115Multiplexer multiplexer) { multiplexer_ = multiplexer; }
void set_gain(ADS1115Gain gain) { gain_ = gain; }
void set_resolution(ADS1115Resolution resolution) { resolution_ = resolution; }
float sample() override;
uint8_t get_multiplexer() const { return multiplexer_; }
uint8_t get_gain() const { return gain_; }
uint8_t get_resolution() const { return resolution_; }

protected:
ADS1115Component *parent_;
ADS1115Multiplexer multiplexer_;
ADS1115Gain gain_;
ADS1115Resolution resolution_;
};


}  // namespace water_quality
}  // namespace esphome

// #endif  // I2C_H