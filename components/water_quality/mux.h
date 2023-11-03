#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include <Wire.h>

namespace esphome {
namespace water_quality {

    // i2c ayarları
    #define SDA 16 
    #define SCL 32
    #define freq 800000

    // i2c adres ayarları
    #define TCA9548_ADDRESS 0x70

class TCA9548
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548
void tcaselect(uint8_t bus){
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

};

}  // namespace water_quality
}  // namespace esphome