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

class MyComponent : public PollingComponent, public i2c::I2CDevice 
{
public:

float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

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

uint16_t AnIn_TempRes = 1000; //temperature sensor model pt1000 and its resistance is 1k
float AnOut_Vcc, AnOut_Temp, TempRes;
uint8_t DigIn_FilterCoeff[4][10];
uint8_t dose, circ;

void setup() override
{

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //  TCA9548

//     Wire.begin(SDA,SCL,freq);

//     for (uint8_t t=0; t<8; t++) 
//     {
//       tcaselect(t);
//       ESP_LOGI(TAG,"TCA Port %d", t);

//       for (uint8_t addr = 0; addr<=127; addr++) 
//       {
//         if (addr == TCA9548_ADDRESS) continue;

//         Wire.beginTransmission(addr);
//         if (!Wire.endTransmission()) 
//         {
//           ESP_LOGI(TAG,"Found I2C 0x%x",addr);
//         }
//       }
//     }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
    
    // tcaselect(0);
    if (!ads1.begin(ADS1X15_ADDRESS1))
    {
      ESP_LOGE(TAG,"Failed to initialize ADS1115_1.");
    //   while (1);
    }
    if (!ads2.begin(ADS1X15_ADDRESS2))
    {
      ESP_LOGE(TAG,"Failed to initialize ADS1115_2.");
    //   while (1);
    }

    // The ADC input range (or gain) can be changed via the following
    // functions, but be careful never to exceed VDD +0.3V max, or to
    // exceed the upper and lower limits if you adjust the input range!
    // Setting these values incorrectly may destroy your ADC!
    
    //                                          ADS1015          ADS1115
    //                                          -------          -------
    // GAIN_TWOTHIRDS  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // GAIN_ONE        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // GAIN_TWO        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // GAIN_FOUR       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // GAIN_EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // GAIN_SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    ads1.setGain(GAIN_TWOTHIRDS);
    ads2.setGain(GAIN_TWOTHIRDS);
    
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

    // tcaselect(0);
    
    if (!mcp.begin_I2C(MCP23008_ADDRESS, &Wire)) 
    {
        ESP_LOGE(TAG,"Failed to initialize MCP23008.");
        // while (1);
    }

    // mcp.pinMode(0, INPUT);
    // mcp.pinMode(1, INPUT);
    // mcp.pinMode(2, INPUT);
    // mcp.pinMode(3, INPUT);
    // mcp.pullUp(0, HIGH);
    // mcp.pullUp(1, HIGH);
    // mcp.pullUp(2, HIGH);
    // mcp.pullUp(3, HIGH);
    // mcp.pinMode(4, OUTPUT);
    // mcp.pinMode(5, OUTPUT);
    // mcp.pinMode(6, OUTPUT);
    // mcp.pinMode(7, OUTPUT);
    // mcp.digitalWrite(4,LOW);
    // mcp.digitalWrite(5,LOW);
    // mcp.digitalWrite(6,LOW);
    // mcp.digitalWrite(7,LOW);
    
    mcp.pinMode(0, INPUT_PULLUP);
    mcp.pinMode(1, INPUT_PULLUP);
    mcp.pinMode(2, INPUT_PULLUP);
    mcp.pinMode(3, INPUT_PULLUP);
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

    // tcaselect(0);
    Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, Wire);
    
    if (!pwm.begin()) 
    {
      ESP_LOGE(TAG,"Failed to initialize PCA9685.");
    //   while (1);
    }
    /*
    * In theory the internal oscillator (clock) is 25MHz but it really isn't
    * that precise. You can 'calibrate' this by tweaking this number until
    * you get the PWM update frequency you're expecting!
    * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
    * is used for calculating things like writeMicroseconds()
    * Analog servos run at ~50 Hz updates, It is importaint to use an
    * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
    * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
    *    the I2C PCA9685 chip you are setting the value for.
    * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
    *    expected value (50Hz for most ESCs)
    * Setting the value here is specific to each individual I2C PCA9685 chip and
    * affects the calculations for the PWM update frequency. 
    * Failure to correctly set the int.osc value will cause unexpected PWM results
    */
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PwmFreq);

}

void loop() override;
void update() override
{
    // if (this->AnInVPow_Val_ != nullptr) 
    // {
    //     this->AnInVPow_Val_->publish_state(AnOut_Vcc);
    // }
}

void dump_config() override
{
    ESP_LOGI(TAG,"Pump_dose = %d", dose);
    ESP_LOGI(TAG,"Pump_circ = %d", circ);

    for (size_t i = 0; i < (dose + circ)*2; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            ESP_LOGI(TAG,"Pump_Calib[%d]-[%d] = %d", i, j, Pump_Calib[i][j]);
        }
    }

    for (size_t i = 0; i < AnInL_LvlResMin.size(); i++)
    {
        ESP_LOGI(TAG,"ResMin[%d] = %d", i, AnInL_LvlResMin[i]);
        ESP_LOGI(TAG,"ResMax[%d] = %d", i, AnInL_LvlResMax[i]);
    }

    ESP_LOGI(TAG,"EC_ch = %d", AnInEC_Ch);
    ESP_LOGI(TAG,"EC_type = %d", AnInEC_Type);
    ESP_LOGI(TAG,"PH_ch = %d", AnInPH_Ch);
    ESP_LOGI(TAG,"PH_type = %d", AnInPH_Type);
}

void pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    dose = d;
    circ = c;
    
    this->Pump_Type = ptype;
}

void pump_calibration(const std::vector<std::vector<uint8_t>> &pcalib)
{ 
    this->Pump_Calib = pcalib;
}

void pump_mode(std::vector<uint8_t> &pmode)
{
    pmode.resize(dose + circ);

    if (Pump_Mode != pmode)
    for (size_t i = 0; i < (dose + circ); i++)
    {
        ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, pmode[i]);
    }

    this->Pump_Mode = pmode;
}

void pump_dose(std::vector<uint8_t> &pdose)
{
    pdose.resize(dose);

    if (Pump_Dose != pdose)
    for (size_t i = 0; i < (dose); i++)
    {
        ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pdose[i]);
    }

    this->Pump_Dose = pdose;
}

void pump_circulation(std::vector<uint16_t> &pcirc)
{
    pcirc.resize(circ);

    if (Pump_Circulation != pcirc)
    for (size_t i = 0; i < (circ); i++)
    {
        ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pcirc[i]);
    }

    this->Pump_Circulation = pcirc;
}

void pump_reset(std::vector<bool> &pres)
{
    pres.resize(dose + circ);

    if (Pump_Reset != pres)
    for (size_t i = 0; i < (dose + circ); i++)
    {
        ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, (int)pres[i]);
    }

    this->Pump_Reset = pres;
}

void servo_mode(std::vector<bool> &smode)
{
    if (Servo_Mode != smode)
    for (size_t i = 0; i < smode.size(); i++)
    {
        ESP_LOGD(TAG,"Servo_Mode[%d] = %d", i, (int)smode[i]);
    }

    this->Servo_Mode = smode;
}

void servo_position(std::vector<uint8_t> &spos)
{
    if (Servo_Position != spos)
    for (size_t i = 0; i < spos.size(); i++)
    {
        ESP_LOGD(TAG,"Servo_Position[%d] = %d", i, spos[i]);
    }
    
    this->Servo_Position = spos;
}

void level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax)
{
    this->AnInL_LvlResMin = rmin;
    this->AnInL_LvlResMax = rmax;
}

void ec(const uint8_t ch, const uint8_t type)
{
    AnInEC_Ch = ch;
    AnInEC_Type = type;
}

void ph(const uint8_t ch, const uint8_t type)
{
    AnInPH_Ch = ch;
    AnInPH_Type = type;
}

void digital_out(std::vector<bool> &dout)
{
    if (DigOut_Status != dout)
    for (size_t i = 0; i < dout.size(); i++)
    {
        ESP_LOGD(TAG,"DigOut_Status[%d] = %d", i, (int)dout[i]);
    }
    
    this->DigOut_Status = dout;
}

void Pump_TimeConstant  (sensor::Sensor *ptc)    { Pump_TimeConstant_ = ptc; }
void Pump_Total_ml      (sensor::Sensor *ptot)   { Pump_Total_ml_ = ptot; }
void Pump_Total_l       (sensor::Sensor *ptot)   { Pump_Total_l_ = ptot; }
void Pump_Status        (sensor::Sensor *pstat)  { Pump_Status_ = pstat; }
void Servo_Status       (sensor::Sensor *servo)  { Servo_Status_ = servo; }
void AnInWT_Val         (sensor::Sensor *wtemp)  { AnInWT_Val_ = wtemp; }
void AnInVPow_Val       (sensor::Sensor *vpow)   { AnInVPow_Val_ = vpow; }
void AnInL_Perc         (sensor::Sensor *level)  { AnInL_Perc_ = level; }
void AnInG_Val          (sensor::Sensor *a)      { AnInG_Val_ = a; }
void AnInEC_Val         (sensor::Sensor *ec)     { AnInEC_Val_ = ec; }
void AnInPH_Val         (sensor::Sensor *ph)     { AnInPH_Val_ = ph; }
void DigIn_Status       (sensor::Sensor *din)    { DigIn_Status_ = din; }

protected:
std::vector<std::vector<uint8_t>> Pump_Calib{};
std::vector<uint8_t> Pump_Type{};
std::vector<uint8_t> Pump_Mode{0,0,0,0,0,0};
std::vector<uint8_t> Pump_Dose{0,0,0,0,0,0};
std::vector<uint16_t> Pump_Circulation{0,0,0,0,0,0};
std::vector<bool> Pump_Reset{0,0,0,0,0,0};
std::vector<bool> Servo_Mode{0,0,0,0,0,0,0,0};
std::vector<uint8_t> Servo_Position{0,0,0,0,0,0,0,0};
std::vector<uint16_t> AnInL_LvlResMin{};
std::vector<uint16_t> AnInL_LvlResMax{};
uint8_t AnInEC_Ch;
uint8_t AnInEC_Type;
uint8_t AnInPH_Ch;
uint8_t AnInPH_Type;
std::vector<bool> DigOut_Status{0,0,0,0};

sensor::Sensor *Pump_TimeConstant_{nullptr};
sensor::Sensor *Pump_Total_ml_{nullptr};
sensor::Sensor *Pump_Total_l_{nullptr};
sensor::Sensor *Pump_Status_{nullptr};
sensor::Sensor *Servo_Status_{nullptr};
sensor::Sensor *AnInWT_Val_{nullptr};
sensor::Sensor *AnInVPow_Val_{nullptr};
sensor::Sensor *AnInL_Perc_{nullptr};
sensor::Sensor *AnInG_Val_{nullptr};
sensor::Sensor *AnInEC_Val_{nullptr};
sensor::Sensor *AnInPH_Val_{nullptr};
sensor::Sensor *DigIn_Status_{nullptr};

};

template<typename... Ts> class PumpModeAction : public Action<Ts...> {
    public:
    PumpModeAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->pump_mode(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class PumpDoseAction : public Action<Ts...> {
    public:
    PumpDoseAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->pump_dose(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class PumpCirculationAction : public Action<Ts...> {
    public:
    PumpCirculationAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint16_t> data = this->val_.value(x...);

    this->parent_->pump_circulation(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint16_t>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class PumpResetAction : public Action<Ts...> {
    public:
    PumpResetAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<bool> data = this->val_.value(x...);

    this->parent_->pump_reset(data);
    }

    TEMPLATABLE_VALUE(std::vector<bool>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class ServoModeAction : public Action<Ts...> {
    public:
    ServoModeAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<bool> data = this->val_.value(x...);

    this->parent_->servo_mode(data);
    }

    TEMPLATABLE_VALUE(std::vector<bool>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class ServoPositionAction : public Action<Ts...> {
    public:
    ServoPositionAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<uint8_t> data = this->val_.value(x...);

    this->parent_->servo_position(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint8_t>, val);

    protected:
    MyComponent *parent_;
};

template<typename... Ts> class DigitalOutAction : public Action<Ts...> {
    public:
    DigitalOutAction(MyComponent *parent) : parent_(parent){};
    
    void play(Ts... x) 
    {
    std::vector<bool> data = this->val_.value(x...);

    this->parent_->digital_out(data);
    }

    TEMPLATABLE_VALUE(std::vector<bool>, val);

    protected:
    MyComponent *parent_;
};

}  // namespace water_quality
}  // namespace esphome