#include "water_quality.h"
#include "water_quality_i2c.h"

    #define ADS1X15_ADDRESS1 0x48
    #define ADS1X15_ADDRESS2 0x49
namespace esphome {
namespace water_quality {

    // I2C i2c;
    Analog an;
    Digital dig;
    Pump pump;
    Servo ser;
    
static unsigned long timepoint = millis();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MyComponent::ADS1115_Setup()
{
    if (!ads1.begin(ADS1X15_ADDRESS1))
        // do
            ESP_LOGE(TAG,"Failed to initialize ADS1115_1.");
        // while (1);
    else
        ESP_LOGI(TAG,"Successfulled to initialize ADS1115_1.");

    if (!ads2.begin(ADS1X15_ADDRESS2))
        // do
            ESP_LOGE(TAG,"Failed to initialize ADS1115_2.");
        // while (1);
    else
        ESP_LOGI(TAG,"Successfulled to initialize ADS1115_2.");

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
    // ads1.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, true);
    // ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, true);

    // AnInEC_Type == 1? EC():EC10();
    // // AnInEC_Type == 10? EC10();

    // ec.begin();
    // ph.begin();
}
void MyComponent::ADS1115_Driver()
{
    float analog_voltage[8];
    for(size_t i = 0; i < 4; i++)
    {
        analog_voltage[i] = ads1.computeVolts(ads1.readADC_SingleEnded(i%4));
        ESP_LOGD(TAG,"ads%d = %f", i+1, analog_voltage[i]);
    }
    for(size_t i = 4; i < 8; i++)
    {
        analog_voltage[i] = ads2.computeVolts(ads2.readADC_SingleEnded(i%4));
        ESP_LOGD(TAG,"ads%d = %f", i+1, analog_voltage[i]);
    }
    // ana.Analog_Input_Driver(analog_voltage);
}

void MyComponent::setup()
{
    // Wire.begin();

    MyComponent::ADS1115_Setup();
    // MCP23008_Setup();

    // PCA9685_Setup();
}
void MyComponent::dump_config()
{
    LOG_I2C_DEVICE(this);
    if (this->is_failed())
        ESP_LOGE(TAG, "Communication failed!");
    else
        ESP_LOGI(TAG, "Communication Successfulled!");
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    // Wire.begin(SDA,SCL,frq);

    for (uint8_t t=0; t<8; t++) 
    {
      tcaselect(t);
      ESP_LOGI(TAG,"TCA Port %d", t);

      for (uint8_t addr = 0; addr<=127; addr++) 
      {
        if (addr == TCA9548_ADDRESS) continue;

        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) 
        {
          ESP_LOGI(TAG,"Found I2C 0x%x",addr);
        }
      }
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ESP_LOGI(TAG,"Pump_dose = %d", pump.dose);
    ESP_LOGI(TAG,"Pump_circ = %d", pump.circ);

    for (size_t i = 0; i < pump.Pump_Type.size(); i++)
    {
        ESP_LOGI(TAG,"Pump_Calib_Gain[%d] = %.2f", i, pump.Pump_Calib_Gain[i]);
    }

    for (size_t i = 0; i < pump.Pump_Type.size(); i++)
    {
        ESP_LOGI(TAG,"Pump_Type[%d] = %d", i, pump.Pump_Type[i]);
        ESP_LOGI(TAG,"Pump_Total[%d] = %d.%d", i, pump.Pump_Total[i][0], pump.Pump_Total[i][1]);
    }

    for (size_t i = 0; i < an.AnInLvl_ResMin.size(); i++)
    {
        ESP_LOGI(TAG,"ResMin[%d] = %d", i, an.AnInLvl_ResMin[i]);
        ESP_LOGI(TAG,"ResMax[%d] = %d", i, an.AnInLvl_ResMax[i]);
    }

    ESP_LOGI(TAG,"EC_ch = %d", an.AnInEC_Ch);
    ESP_LOGI(TAG,"EC_type = %d", an.AnInEC_Type);
    ESP_LOGI(TAG,"PH_ch = %d", an.AnInPH_Ch);
    ESP_LOGI(TAG,"PH_type = %d", an.AnInPH_Type);
}
void MyComponent::loop() 
{
    // delay(1000);
}
void MyComponent::update()
{
    // ADS1115_Driver();
    // MCP23008_Driver();
    pca9685();
    // pump_total();
    sensor();
}

    bool pd[6], pc[6];

void MyComponent::pump_calib_gain(const std::vector<float> &pcg)
{
    pump.Pump_Calib_Gain = pcg;
}
void MyComponent::pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    pump.dose = d;
    pump.circ = c;
    
    pump.Pump_Type = ptype;
}
void MyComponent::pump_dose(std::vector<uint16_t> &pdose)
{
    if (pump.Pump_Dose != pdose)
    {
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            if (pump.Pump_Status[i] != 1)
                pump.Pump_Dose[i] = pdose[i];
            ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pump.Pump_Dose[i]);
        }
    }
}
void MyComponent::pump_circulation(std::vector<uint16_t> &pcirc)
{
    if (pump.Pump_Circulation != pcirc)
    {
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            if (pump.Pump_Status[i] != 1)
                pump.Pump_Dose[i] = pcirc[i];
            ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pump.Pump_Circulation[i]);
        }
    }
}
void MyComponent::pump_mode(std::vector<uint8_t> &pmode)
{
    if (pump.Pump_Mode != pmode)
    {
        pump.Pump_Mode = pmode;
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, pump.Pump_Mode[i]);
        
            if (pmode[i] == 1)
            {
                pump.Pump_Status[i] = 1;
                pump.pump_total();
            }
        }
    }
}
void MyComponent::pump_reset(std::vector<bool> &pres)
{
    if (pump.Pump_Reset != pres)
    {
        pump.Pump_Reset = pres;
        for (size_t i = 0; i < pump.Pump_Type.size(); i++)
        {
            if (pump.Pump_Reset[i])
            {
                pump.Pump_Total[i][0] = 0;
                pump.Pump_Total[i][1] = 0;
            }
            ESP_LOGD(TAG,"Pump_Total[%d] = %d.%d", i, pump.Pump_Total[i][0], pump.Pump_Total[i][1]);
            ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, (int)pump.Pump_Reset[i]);
        }
    }
}
void MyComponent::servo_mode(std::vector<bool> &smode)
{
    if (ser.Servo_Mode != smode)
    {
        ser.Servo_Mode = smode;
        for (size_t i = 0; i < ser.Servo_Mode.size(); i++)
        {
            ESP_LOGD(TAG,"Servo_Mode[%d] = %d", i, (int)ser.Servo_Mode[i]);
        }
    }
}
void MyComponent::servo_position(std::vector<uint8_t> &spos)
{
    if (ser.Servo_Position != spos)
    {
        ser.Servo_Position = spos;
        for (size_t i = 0; i < ser.Servo_Position.size(); i++)
        {
            ESP_LOGD(TAG,"Servo_Position[%d] = %d", i, ser.Servo_Position[i]);
        }
    }
}
void MyComponent::level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax)
{
    an.AnInLvl_ResMin = rmin;
    an.AnInLvl_ResMax = rmax;
}
void MyComponent::ec(const uint8_t ch, const uint8_t type)
{
    an.AnInEC_Ch = ch;
    an.AnInEC_Type = type;
}
void MyComponent::ph(const uint8_t ch, const uint8_t type)
{
    an.AnInPH_Ch = ch;
    an.AnInPH_Type = type;
}
void MyComponent::digital_out(std::vector<bool> &dout)
{
    if (dig.DigOut_Status != dout)
    {
        dig.DigOut_Status = dout;
        for (size_t i = 0; i < dig.DigOut_Status.size(); i++)
        {
            ESP_LOGD(TAG,"DigOut_Status[%d] = %d", i, (int)dig.DigOut_Status[i]);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor
void MyComponent::sensor()
{
    std::stringstream pt;
    std::stringstream ps;
    std::stringstream ss;
    std::stringstream ap;
    std::stringstream av;
    std::stringstream ds;

    if (this->Pump_Tot_ != nullptr)
    { 
        for (size_t i = 0; i < 6; i++)
        if (i > 0)
        pt << "," << std::fixed << std::setprecision(3) << (float)pump.Pump_Total[i][0] + (float)pump.Pump_Total[i][1]/1000;
        else
        pt << std::fixed << std::setprecision(3) << (float)pump.Pump_Total[i][0] + (float)pump.Pump_Total[i][1]/1000;
    
        this->Pump_Tot_->publish_state(pt.str());
    }
    if (this->Pump_Stat_ != nullptr)
    { 
        for (size_t i = 0; i < 6; i++)
        if (i > 0)
        ps << "," << std::fixed << std::setprecision(0) << (int)pump.Pump_Status[i];
        else
        ps << std::fixed << std::setprecision(0) << (int)pump.Pump_Status[i];

        this->Pump_Stat_->publish_state(ps.str());
    }
    if (this->Servo_Stat_ != nullptr)
    { 
        for (size_t i = 0; i < 8; i++)
        if (i > 0)
        ss << "," << std::fixed << std::setprecision(0) << (int)ser.Servo_Status[i];
        else
        ss << std::fixed << std::setprecision(0) << (int)ser.Servo_Status[i];

        this->Servo_Stat_->publish_state(ss.str());
    }
    if (this->AnInWT_Val_ != nullptr) { this->AnInWT_Val_->publish_state(an.get_WT()); }
    if (this->AnInVPow_Val_ != nullptr) { this->AnInVPow_Val_->publish_state(an.VPow); }
    if (this->AnInLvl_Perc_ != nullptr) 
    {
        for (size_t i = 0; i < 2; i++)
        if (i > 0)
        ap << "," << std::fixed << std::setprecision(2) << an.LvlPerc[i];
        else
        ap << std::fixed << std::setprecision(2) << an.LvlPerc[i];

        this->AnInLvl_Perc_->publish_state(ap.str());
    }
    if (this->AnInEC_Val_ != nullptr) { this->AnInEC_Val_->publish_state(an.EC); }
    if (this->AnInPH_Val_ != nullptr) { this->AnInPH_Val_->publish_state(an.PH); }
    if (this->AnInGen_Val_ != nullptr) 
    {
        for (size_t i = 0; i < 2; i++)
        if (i > 0)
        av << "," << std::fixed << std::setprecision(2) << an.AnGen[i];
        else
        av << std::fixed << std::setprecision(2) << an.AnGen[i];
    
        this->AnInGen_Val_->publish_state(av.str());
    }
    if (this->DigIn_Stat_ != nullptr) 
    {
        for (size_t i = 0; i < 4; i++)
        if (i > 0)
        ds << "," << std::fixed << std::setprecision(2) << (int)dig.DigIn_Status[i];
        else
        ds << std::fixed << std::setprecision(2) << (int)dig.DigIn_Status[i];

        this->DigIn_Stat_->publish_state(ds.str());
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace water_quality
}  // namespace esphome