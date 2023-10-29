#include "water_quality.h"

namespace esphome {
namespace water_quality {

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

    void tcaselect(uint8_t bus)
    {
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
    }

void MyComponent::setup() 
{
    
}

void MyComponent::loop() 
{
    
}

void MyComponent::update() 
{
    if (this->AnInVPow_Val_ != nullptr) 
    {
        this->AnInVPow_Val_->publish_state(AnOut_Vcc);
    }
}

}  // namespace water_quality
}  // namespace esphome