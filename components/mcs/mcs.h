#pragma once

#ifndef MCS_H
#define MCS_H

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
#include "mcs_i2c.h"
#include <esp_timer.h>

#define MCS_EEPROM_SIZE 6 // byte
#define BUTTON_ADDR 0x00
#define LED_L_ADDR 0x02
#define LED_R_ADDR 0x04

namespace esphome {
namespace mcs {

static const char *const TAG = "mcs";

enum MCP23017_Registers 
{
    // A side
    MCP23017_IODIRA = 0x00,
    MCP23017_IPOLA = 0x02,
    MCP23017_GPINTENA = 0x04,
    MCP23017_DEFVALA = 0x06,
    MCP23017_INTCONA = 0x08,
    MCP23017_IOCONA = 0x0A,
    MCP23017_GPPUA = 0x0C,
    MCP23017_INTFA = 0x0E,
    MCP23017_INTCAPA = 0x10,
    MCP23017_GPIOA = 0x12,
    MCP23017_OLATA = 0x14,
    // B side
    MCP23017_IODIRB = 0x01,
    MCP23017_IPOLB = 0x03,
    MCP23017_GPINTENB = 0x05,
    MCP23017_DEFVALB = 0x07,
    MCP23017_INTCONB = 0x09,
    MCP23017_IOCONB = 0x0B,
    MCP23017_GPPUB = 0x0D,
    MCP23017_INTFB = 0x0F,
    MCP23017_INTCAPB = 0x11,
    MCP23017_GPIOB = 0x13,
    MCP23017_OLATB = 0x15,
};
enum MCP23017_PinMode
{
    FLAG_NONE,
    FLAG_INPUT,
    FLAG_OUTPUT,
    FLAG_OPEN_DRAIN,
    FLAG_PULLUP,
    FLAG_PULLDOWN,
};
enum MCP23017_InterruptMode : uint8_t
{
    MCP23017_NO_INTERRUPT = 0,
    MCP23017_CHANGE,
    MCP23017_RISING,
    MCP23017_FALLING,
};

class MCS_I2C;

class MCS : public PollingComponent, public i2c::I2CDevice
{
public:
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
//  MCP23017

void MCP23017_Setup(uint8_t address);
void MCP23017_Write(bool value[], uint8_t state);
void MCP23017_Driver(bool digital[]);
void MCP23017_pin_interrupt_mode(uint8_t pin, MCP23017_InterruptMode interrupt_mode);

void set_open_drain_ints(const bool value)                      { this->open_drain_ints_ = value; }
void set_pin(uint8_t pin)                                       { pin_ = pin; }
void set_inverted(bool inverted)                                { inverted_ = inverted; }
void set_interrupt_mode(MCP23017_InterruptMode interrupt_mode)  { interrupt_mode_ = interrupt_mode; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void version(const uint8_t ver);
void digital_out(std::vector<bool> &dout);
void digital_out2(uint8_t dout);

protected:
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23017

uint16_t olat_a1_{0x01};
uint16_t olat_a2_{0x01};
uint16_t olat_b_{0x00};
uint8_t pin_;
bool inverted_;
MCP23017_InterruptMode interrupt_mode_;
bool open_drain_ints_;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
esp_timer_handle_t timer;
};
template<typename... Ts> class Digital_Out_Action : public Action<Ts...> {
public:
Digital_Out_Action(MCS *parent) : parent_(parent){};

TEMPLATABLE_VALUE(std::vector<bool>, dig_out);

void play(Ts... x) 
{
    std::vector<bool> data = this->dig_out_.value(x...);

    this->parent_->digital_out(data);
}

protected:
MCS *parent_;
};
template<typename... Ts> class Digital_Out_Action2 : public Action<Ts...> {
public:
Digital_Out_Action2(MCS *parent) : parent_(parent){};

TEMPLATABLE_VALUE(uint8_t, dig_out2);

void play(Ts... x) 
{
    uint8_t data = this->dig_out2_.value(x...);

    this->parent_->digital_out2(data);
}

protected:
MCS *parent_;
};

}  // namespace mcs
}  // namespace esphome

#endif  // MCS_H