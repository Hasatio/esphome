#include "component.h"

namespace esphome {
namespace water_quality_control {

static const char *TAG = "mysensor";

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

    Adafruit_MCP23008 mcp;

    Adafruit_PWMServoDriver pwm;


    // i2c ayarları
    #define SDA 21 
    #define SCL 22
    #define freq 800000

    // i2c adres ayarları
    #define ADS1X15_ADDRESS1 0x48
    #define ADS1X15_ADDRESS2 0x49
    #define MCP23008_ADDRESS 0x20
    #define PCA9685_I2C_ADDRESS 0x40 


    uint16_t adc[8], pwmfreq=1000;
    uint64_t sayac = 0;
    float volts[8];

void Component::setup() 
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  i2c
    
    Wire.begin(SDA,SCL,freq);
    // Wire.setClock(800000);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
  
    bool status1 = ads1.begin(ADS1X15_ADDRESS1,&Wire);
    bool status2 = ads2.begin(ADS1X15_ADDRESS2,&Wire);

    if (!status1)
    {
      ESP_LOGD(TAG,"Failed to initialize ADS1115_1.");
      while (1);
    }
    if (!status2)
    {
      ESP_LOGD(TAG,"Failed to initialize ADS1115_2.");
      while (1);
    }

    // The ADC input range (or gain) can be changed via the following
    // functions, but be careful never to exceed VDD +0.3V max, or to
    // exceed the upper and lower limits if you adjust the input range!
    // Setting these values incorrectly may destroy your ADC!
    //
    //                                          ADS1015          ADS1115
    //                                          -------          -------
    // GAIN_TWOTHIRDS  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // GAIN_ONE        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // GAIN_TWO        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // GAIN_FOUR       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // GAIN_EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // GAIN_SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    ads1.setGain(GAIN_ONE); 
    ads2.setGain(GAIN_ONE); 
    
    // RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
    // RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
    // RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
    // RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
    // RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
    // RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
    // RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
    // RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second
    ads1.setDataRate(RATE_ADS1115_860SPS);
    ads2.setDataRate(RATE_ADS1115_860SPS);
    
    // ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
    // ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000) ///< Differential P = AIN0, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000) ///< Differential P = AIN1, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000) ///< Differential P = AIN2, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
    // ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
    // ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
    // ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3
    // ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

    bool status3 = mcp.begin(MCP23008_ADDRESS,&Wire);

    if (!status3)
    {
      ESP_LOGD(TAG,"Failed to initialize MCP23008.");
      while (1);
    }

    mcp.pinMode(0, INPUT);
    mcp.pinMode(1, INPUT);
    mcp.pinMode(2, INPUT);
    mcp.pinMode(3, INPUT);
    mcp.pullUp(0, HIGH);
    mcp.pullUp(1, HIGH);
    mcp.pullUp(2, HIGH);
    mcp.pullUp(3, HIGH);
    mcp.pinMode(4, OUTPUT);
    mcp.pinMode(5, OUTPUT);
    mcp.pinMode(6, OUTPUT);
    mcp.pinMode(7, OUTPUT);
    mcp.digitalWrite(4,LOW);
    mcp.digitalWrite(5,LOW);
    mcp.digitalWrite(6,LOW);
    mcp.digitalWrite(7,LOW);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685

    Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS,&Wire);
    
    pwm.begin();

    pwm.setPWMFreq(pwmfreq);

}

void Component::loop() 
{
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    for(int i=0;i<4;i++)
    {
      adc[i] = ads1.readADC_SingleEnded(i%4);
      volts[i] = ads1.computeVolts(adc[i]) * mygain;
      data = data + String(volts[i]) + ",";
    }
    for(int i=4;i<8;i++)
    {
      adc[i] = ads2.readADC_SingleEnded(i%4);
      volts[i] = ads2.computeVolts(adc[i]) * mygain;
      data = data + String(volts[i]) + ",";
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

    mcp.digitalWrite(4,LOW);
    mcp.digitalWrite(5,LOW);
    mcp.digitalWrite(6,LOW);
    mcp.digitalWrite(7,LOW);

    if (mcp.digitalRead(0) == LOW)
    {
        mcp.digitalWrite(4, HIGH);
    }  
    if (mcp.digitalRead(1) == LOW)
    {
        mcp.digitalWrite(5, HIGH);
    }
    if (mcp.digitalRead(2) == LOW)
    {
        mcp.digitalWrite(6, HIGH);
    }  
    if (mcp.digitalRead(3) == LOW)
    {
        mcp.digitalWrite(7, HIGH);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685

    for (uint8_t pin=0; pin<16; pin++) 
    {
    pwm.setPWM(pin, 4096, 0);       // turns pin fully on
    delay(100);
    pwm.setPWM(pin, 0, 4096);       // turns pin fully off
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Sensor
    
if (this->sample_ != nullptr) this->sample_->publish_state(sayac);
if (this->sample_sec_ != nullptr) this->sample_sec_->publish_state(sayac*1000/millis());

}


}  // namespace water_quality_control
}  // namespace esphome