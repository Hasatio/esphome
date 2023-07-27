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

class Myi2cComponent : public Component, public Sensor 
{
public:
    float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

    Sensor *accel_x_sensor = new Sensor();
    Sensor *accel_y_sensor = new Sensor();
    Sensor *accel_z_sensor = new Sensor();
    Sensor *voltage_sensor = new Sensor();
    Sensor *percentage_sensor = new Sensor();
    Sensor *sayi = new Sensor();
    
    int sayac = 0;
    float x = 0.0;
    char *btname = "ESP32";

    int16_t adc[16];
    float volts[16], x, y, z, voltage, percentage;
    char *data = {};
    int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7, adc8, adc9, adc10, adc11, adc12, adc13, adc14, adc15;
    float volts0, volts1, volts2, volts3, volts4, volts5, volts6, volts7, volts8, volts9, volts10, volts11, volts12, volts13, volts14, volts15;

void device(char *name);

void gain(float g);
    
void setup() override;

void loop() override;

};
} //namespace myi2c
} //namespace esphome
