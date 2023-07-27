#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/time.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MAX1704X.h>
#include <BluetoothSerial.h>

namespace esphome {
namespace myi2c {

BluetoothSerial SerialBT;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth off--Run `make menuconfig` to enable it
#endif

//Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;
Adafruit_ADS1115 ads4;
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

Adafruit_MAX17048 maxlipo;

#define SDA 21
#define SCL 22
#define freq 800000
/*
#define SDA_1 32
#define SCL_1 33
#define freq_1 800000

#define SDA_2 25
#define SCL_2 26
#define freq_2 800000

#define SDA_3 27
#define SCL_3 14
#define freq_3 800000

#define SDA_4 9
#define SCL_4 10
#define freq_4 800000
*/
#define address1 0x48
#define address2 0x49
#define address3 0x4a
#define address4 0x4b

TwoWire I2C_1 = TwoWire(0);
//TwoWire I2C_2 = TwoWire(1);

class Myi2c : public Component, public Sensor 
{
public:

   float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void device(char *name);

void gain(float g);
    
void setup() override;

void loop() override;

};
} //namespace myi2c
} //namespace esphome
