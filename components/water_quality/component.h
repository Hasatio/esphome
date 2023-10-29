#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"
#include <array>
#include <vector>
#include <iterator>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include "Adafruit_MCP23X08.h"
#include <Adafruit_PWMServoDriver.h>

namespace esphome {
namespace water_quality {

static const char *const TAG = "component";

class MyComponent
{
public:

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

    Adafruit_MCP23X08 mcp;

    Adafruit_PWMServoDriver pwm;


    // i2c ayarları
    #define SDA 16 
    #define SCL 32
    #define freq 800000

    // i2c adres ayarları
    #define TCA9548_ADDRESS 0x70 
    #define ADS1X15_ADDRESS1 0x48
    #define ADS1X15_ADDRESS2 0x49
    #define MCP23008_ADDRESS 0x20
    #define PCA9685_I2C_ADDRESS 0x40 


    uint16_t adc[8], PwmFreq = 1000;
    float volts[8];

    uint16_t AnInWT_Res = 1000; //temperature sensor model pt1000 and its resistance is 1k
    float VPow, WT, WT_Res;
    uint16_t AnOut_SensPerc[4];

    uint8_t DigIn_FilterCoeff[4][10];
    bool DigIn_Status[4], DigOut_Status[4]; 

uint8_t dose, circ;


void tcaselect(uint8_t bus){
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
void ads1115()
{
    tcaselect(0);
    for(int i = 0; i < 4; i++)
    {
        adc[i] = ads1.readADC_SingleEnded(i%4);
        volts[i] = ads1.computeVolts(adc[i]);
        // ESP_LOGD(TAG,"ads%d = %f", i+1, volts[i]);
    }
    for(int i = 4; i < 8; i++)
    {
        adc[i] = ads2.readADC_SingleEnded(i%4);
        volts[i] = ads2.computeVolts(adc[i]);
        // ESP_LOGD(TAG,"ads%d = %f", i+1, volts[i]);
    }

    WT_Res = (float)(volts[0] * 1000) / (5 - volts[0]) * (AnInWT_Res / 1000); //R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
    WT = (float)(sqrt((-0.00232 * WT_Res) + 17.59246) - 3.908) / (-0.00116)  ; //Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116
    VPow = (float)volts[1] * 6; //Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k

    // AnOut_LvlPerc[0] = (int)volts[2] * 100 / 5 * AnIn_LvlResMax[0] / (1000 + AnIn_LvlResMax[0]) - 5 * AnIn_LvlResMin[0] / (1000 + AnIn_LvlResMin[0]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    // AnOut_LvlPerc[1] = (int)volts[3] * 100 / 5 * AnIn_LvlResMax[1] / (1000 + AnIn_LvlResMax[1]) - 5 * AnIn_LvlResMin[1] / (1000 + AnIn_LvlResMin[1]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
}

};

}  // namespace water_quality
}  // namespace esphome