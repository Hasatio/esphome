#pragma once

#ifndef MUX_H
#define MUX_H

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>

    // i2c ayarları
    #define SDA 16 
    #define SCL 32
    #define frq 800000

    // i2c adres ayarları
    #define TCA9548_ADDRESS 0x70
    #define ADS1X15_ADDRESS1 0x48
    #define ADS1X15_ADDRESS2 0x49
    #define MCP23008_ADDRESS 0x20
    #define PCA9685_I2C_ADDRESS 0x40

namespace esphome {
namespace water_quality {

class Mux
{
public:
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548
void tcaselect(uint8_t bus){
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

};

}  // namespace water_quality
}  // namespace esphome

#endif  // MUX_H