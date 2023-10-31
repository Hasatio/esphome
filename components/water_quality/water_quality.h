#pragma once

// #include "component.h"
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

static const char *const TAG = "mycomponent";

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548
void tcaselect(uint8_t bus){
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
void ads1115_set()
{ 
    tcaselect(0);
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
}
void ads1115()
{
    tcaselect(0);
    for(size_t i = 0; i < 4; i++)
    {
        adc[i] = ads1.readADC_SingleEnded(i%4);
        volts[i] = ads1.computeVolts(adc[i]);
        // ESP_LOGD(TAG,"ads%d = %f", i+1, volts[i]);
    }
    for(size_t i = 4; i < 8; i++){
        adc[i] = ads2.readADC_SingleEnded(i%4);
        volts[i] = ads2.computeVolts(adc[i]);
        // ESP_LOGD(TAG,"ads%d = %f", i+1, volts[i]);
    }

    WT_Res = (float)(volts[0] * 1000) / (5 - volts[0]) * (AnInWT_Res / 1000); //R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
    WT = (float)(sqrt((-0.00232 * WT_Res) + 17.59246) - 3.908) / (-0.00116)  ; //Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116
    VPow = (float)volts[1] * 6; //Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k

    LvlPerc[0] = (float)volts[2] * 100 / 5 * AnInLvl_ResMax[0] / (1000 + AnInLvl_ResMax[0]) - 5 * AnInLvl_ResMin[0] / (1000 + AnInLvl_ResMin[0]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    LvlPerc[1] = (float)volts[3] * 100 / 5 * AnInLvl_ResMax[1] / (1000 + AnInLvl_ResMax[1]) - 5 * AnInLvl_ResMin[1] / (1000 + AnInLvl_ResMin[1]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008
void mcp23008_set()
{
    tcaselect(0);
    
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
}
void mcp23008()
{
    tcaselect(0);
    mcp.digitalWrite(4,LOW);
    mcp.digitalWrite(5,LOW);
    mcp.digitalWrite(6,LOW);
    mcp.digitalWrite(7,LOW);

    for(size_t i = 0; i < 4; i++)
    {
        DigIn_Read[i] = mcp.digitalRead(i);
        // ESP_LOGD(TAG,"dig input %d = %d", i, DigIn_Read[i]);
    }

    for(size_t i = 0; i < 4; i++)
    {
        if (DigOut_Status[i] == 1)
        {
            mcp.digitalWrite(i + 4, HIGH);
        }
        else
        { 
            mcp.digitalWrite(i + 4, LOW);
        }
        // ESP_LOGD(TAG,"dig output %d = %d", i, DigOut_Status[i]);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685
void pca9685_set()
{
    tcaselect(0);
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
void pca9685()
{
    tcaselect(0);
    // for (uint8_t pin=0; pin<16; pin++) 
    // {
    // pwm.setPWM(pin, 4096, 0);       // turns pin fully on
    // delay(100);
    // pwm.setPWM(pin, 0, 4096);       // turns pin fully off
    // }
    // for (uint16_t i=0; i<4096; i += 8) 
    // {
    //     for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) 
    //     {
    //     pwm.setPWM(pwmnum, 0, (i + (4096/16)*pwmnum) % 4096 );
    //     }
    // }
    // for (uint16_t i=0; i<4096; i += 8) 
    // {
    //     for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) 
    //     {
    //     pwm.setPin(pwmnum, (i + (4096/16)*pwmnum) % 4096 );
    //     }
        
    //     ESP_LOGD(TAG,"pwm = %d", i);
    // }

    // for (uint8_t i = 0; i < Pump_Total[1].size(); i++) 
    //     {
    //         Pump_Total[1][i] += i;
    //     }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor
void sensor()
{
    if (this->Pump_Total_l_ != nullptr) { 
        for (size_t i = 0; i < sizeof(AnInLvl_Perc_); i++)
        this->Pump_Total_l_->publish_state(Pump_Total[0][i]);
    }
    if (this->Pump_Total_ml_ != nullptr) { 
        for (size_t i = 0; i < sizeof(AnInLvl_Perc_); i++)
        this->Pump_Total_ml_->publish_state(Pump_Total[1][i]);
    }

    if (this->AnInWT_Val_ != nullptr) { this->AnInWT_Val_->publish_state(WT); }
    if (this->AnInVPow_Val_ != nullptr) { this->AnInVPow_Val_->publish_state(VPow); }
    if (this->AnInLvl_Perc_ != nullptr) { 
        for (size_t i = 0; i < sizeof(AnInLvl_Perc_); i++) 
        this->AnInLvl_Perc_->publish_state(LvlPerc[i]);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() override
{
    ads1115_set();
    mcp23008_set();
    pca9685_set();
}
void loop() override;
// void dump_config() override
// {
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
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//     ESP_LOGI(TAG,"Pump_dose = %d", dose);
//     ESP_LOGI(TAG,"Pump_circ = %d", circ);

//     for (size_t i = 0; i < Pump_Type.size(); i++)
//     {
//         for (size_t j = 0; j < 8; j++)
//         {
//             ESP_LOGI(TAG,"Pump_Calib_X[%d]-[%d] = %d", i, j, Pump_Calib[2*i][j]);
//         }
//         for (size_t j = 0; j < 8; j++)
//         {
//             ESP_LOGI(TAG,"Pump_Calib_Y[%d]-[%d] = %d", i, j, Pump_Calib[2*i+1][j]);
//         }
//     }

//     for (size_t i = 0; i < Pump_Type.size(); i++)
//     {
//         ESP_LOGI(TAG,"Pump_Type[%d] = %d", i, Pump_Type[i]);
//         // ESP_LOGI(TAG,"Pump_Total[%d] = %d.%d", i, Pump_Total[i][0], Pump_Total[i][1]);
//     }

//     for (size_t i = 0; i < AnInLvl_ResMin.size(); i++)
//     {
//         ESP_LOGI(TAG,"ResMin[%d] = %d", i, AnInLvl_ResMin[i]);
//         ESP_LOGI(TAG,"ResMax[%d] = %d", i, AnInLvl_ResMax[i]);
//     }

//     ESP_LOGI(TAG,"EC_ch = %d", AnInEC_Ch);
//     ESP_LOGI(TAG,"EC_type = %d", AnInEC_Type);
//     ESP_LOGI(TAG,"PH_ch = %d", AnInPH_Ch);
//     ESP_LOGI(TAG,"PH_type = %d", AnInPH_Type);
// }
void update() override;


void pump_time_constant(const std::vector<std::string> &ptc)
{
    this->Pump_Time_Constant = ptc;
}
void pump_calibration(const std::vector<std::vector<uint8_t>> &pcalib)
{
    this->Pump_Calib = pcalib;
}
void pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    dose = d;
    circ = c;
    
    this->Pump_Type = ptype;
}
void pump_dose(std::vector<uint16_t> &pdose)
{
    // pdose.resize(dose);

    if (Pump_Dose != pdose)
    for (size_t i = 0; i < (6); i++)
    {
        ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pdose[i]);
    }

    this->Pump_Dose = pdose;
}
void pump_circulation(std::vector<uint16_t> &pcirc)
{
    // pcirc.resize(circ);

    if (Pump_Circulation != pcirc)
    for (size_t i = 0; i < (6); i++)
    {
        ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pcirc[i]);
    }

    this->Pump_Circulation = pcirc;
}
// void pump_total()
// {
//     for (size_t i = 0; i < 6; i++)
//     {
//         if (Pump_Mode[i] == 1)
//         {
//             if (Pump_Type[i] == 1)
//             {
//                 while (Pump_Dose[i] >= 1)
//                 {
//                     ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, Pump_Dose[i]);
//                     Pump_Total[i][1] = Pump_Dose[i]%2;
//                     Pump_Total[i][0] += (int)(Pump_Total[i][0] + Pump_Dose[i])/1000;
//                     Pump_Total[i][1] = (int)(Pump_Total[i][1] + Pump_Dose[i])%1000;
//                     if (Pump_Dose[i] == 1)
//                     Pump_Dose[i] = 0;
//                     else
//                     Pump_Dose[i] /= 2;
//                 }
//                 Pump_Mode[i] = 0;
//             }
//                 Pump_Total[1][i] = Pump_Dose[i];
//             if (Pump_Type[i] == 2)
//                 Pump_Total[1][i] = Pump_Circulation[i];
//             // ESP_LOGD(TAG,"Pump_Total[%d] = %d.%d", i, Pump_Total[i][0], Pump_Total[i][1]);
//         }
//     }
// }
void pump_mode(std::vector<uint8_t> &pmode)
{
    if (Pump_Mode != pmode)
    for (size_t i = 0; i < (6); i++)
    {
        ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, pmode[i]);
    }
    this->Pump_Mode = pmode;
}
void pump_reset(std::vector<bool> &pres)
{
    pres.resize(dose + circ);

    if (Pump_Reset != pres)
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < (dose + circ); j++)
        {
            if (!pres[j])
            {
                Pump_Total[i][j] = 0;
            }
        }
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
    this->AnInLvl_ResMin = rmin;
    this->AnInLvl_ResMax = rmax;
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

void Pump_Total_ML      (sensor::Sensor *ptot)   { Pump_Total_ml_ = ptot; }
void Pump_Total_L       (sensor::Sensor *ptot)   { Pump_Total_l_ = ptot; }
void Pump_Status        (sensor::Sensor *pstat)  { Pump_Status_ = pstat; }
void Servo_Status       (sensor::Sensor *servo)  { Servo_Status_ = servo; }
void AnInWT_Val         (sensor::Sensor *wtemp)  { AnInWT_Val_ = wtemp; }
void AnInVPow_Val       (sensor::Sensor *vpow)   { AnInVPow_Val_ = vpow; }
void AnInLvl_Perc       (sensor::Sensor *level)  { AnInLvl_Perc_ = level; }
void AnInGlob_Val       (sensor::Sensor *a)      { AnInGlob_Val_ = a; }
void AnInEC_Val         (sensor::Sensor *ec)     { AnInEC_Val_ = ec; }
void AnInPH_Val         (sensor::Sensor *ph)     { AnInPH_Val_ = ph; }
void DigIn_Status       (sensor::Sensor *din)    { DigIn_Status_ = din; }

protected:
std::vector<std::string> Pump_Time_Constant{};
std::vector<std::vector<uint8_t>> Pump_Calib{};
std::vector<uint8_t> Pump_Type{};
uint8_t dose, circ;
std::vector<uint8_t> Pump_Mode{0, 0, 0, 0, 0, 0};
std::vector<bool> Pump_Reset{0, 0, 0, 0, 0, 0};
std::vector<uint16_t> Pump_Dose{0, 0, 0, 0, 0, 0};
std::vector<uint16_t> Pump_Circulation{0, 0, 0, 0, 0, 0};
std::vector<std::vector<uint16_t>> Pump_Total{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
std::vector<bool> Servo_Mode{0, 0, 0, 0, 0, 0, 0, 0};
std::vector<uint8_t> Servo_Position{0, 0, 0, 0, 0, 0, 0, 0};
float VPow, WT, WT_Res;
uint16_t AnInWT_Res = 1000; //temperature sensor model pt1000 and its resistance is 1k
uint16_t adc[8], PwmFreq = 1000;
float volts[8];
std::vector<uint16_t> AnInLvl_ResMin{0};
std::vector<uint16_t> AnInLvl_ResMax{0};
std::vector<float> LvlPerc{0};
uint8_t AnInEC_Ch;
uint8_t AnInEC_Type;
uint8_t AnInPH_Ch;
uint8_t AnInPH_Type;
std::vector<std::vector<uint8_t>> DigIn_FilterCoeff{0};
std::vector<bool> DigIn_Read{0,0,0,0};
std::vector<bool> DigOut_Status{0,0,0,0};

sensor::Sensor *Pump_Total_ml_{nullptr};
sensor::Sensor *Pump_Total_l_{nullptr};
sensor::Sensor *Pump_Status_{nullptr};
sensor::Sensor *Servo_Status_{nullptr};
sensor::Sensor *AnInWT_Val_{nullptr};
sensor::Sensor *AnInVPow_Val_{nullptr};
sensor::Sensor *AnInLvl_Perc_{nullptr};
sensor::Sensor *AnInGlob_Val_{nullptr};
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
    std::vector<uint16_t> data = this->val_.value(x...);

    this->parent_->pump_dose(data);
    }

    TEMPLATABLE_VALUE(std::vector<uint16_t>, val);

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