#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_analog.h"
#include "wq_digital.h"
#include "wq_pump.h"
#include "wq_servo.h"

namespace esphome {
namespace water_quality {

// WaterQuality water_quality_instance;
// WQ_I2C i2c(&water_quality_instance);

//     // WQ_I2C i2c;
    Analog an;
    Digital dig;
    Pump pump;
    Servo ser;

// WaterQuality::WaterQuality() { wq_i2c_ = new WQ_I2C(this); }

void EEPROM_write(int address, float value)
{
    byte *data = (byte*)&(value);
    for (int i = 0; i < sizeof(value); i++)
        EEPROM.write(address + i, data[i]);
}
float EEPROM_read(int address)
{
    float value = 0.0;
    byte *data = (byte*)&(value);
    for (int i = 0; i < sizeof(value); i++)
        data[i] = EEPROM.read(address + i);
    return value;
}

void PH_Setup()
{
    float acidVoltage = 2032.44; // mV
    float neutralVoltage = 1500; // mV
    float baseVoltage = 967.56; // mV

    float acidPh = 4.0;
    float neutralPh = 7.0;
    float basePh = 10.0;
    
    EEPROM.begin(EEPROM_SIZE);

    if (EEPROM.read(PH1ADDR) == 0xFF && EEPROM.read(PH1ADDR + 1) == 0xFF && EEPROM.read(PH1ADDR + 2) == 0xFF && EEPROM.read(PH1ADDR + 3) == 0xFF)
        EEPROM_write(PH1ADDR, neutralPh); // New EEPROM, write typical pH value
    if (EEPROM.read(Volt1ADDR) == 0xFF && EEPROM.read(Volt1ADDR + 1) == 0xFF && EEPROM.read(Volt1ADDR + 2) == 0xFF && EEPROM.read(Volt1ADDR + 3) == 0xFF)
        EEPROM_write(Volt1ADDR, neutralVoltage); // New EEPROM, write typical pH voltage
    if (EEPROM.read(PH2ADDR) == 0xFF && EEPROM.read(PH2ADDR + 1) == 0xFF && EEPROM.read(PH2ADDR + 2) == 0xFF && EEPROM.read(PH2ADDR + 3) == 0xFF)
        EEPROM_write(PH2ADDR, acidPh); // New EEPROM, write typical pH value
    if (EEPROM.read(Volt2ADDR) == 0xFF && EEPROM.read(Volt2ADDR + 1) == 0xFF && EEPROM.read(Volt2ADDR + 2) == 0xFF && EEPROM.read(Volt2ADDR + 3) == 0xFF)
        EEPROM_write(Volt2ADDR, acidVoltage); // New EEPROM, write typical pH voltage
    
    float eepromPH1 = EEPROM_read(PH1ADDR); // Load the value of the pH board from the EEPROM
    float eepromVolt1 = EEPROM_read(Volt1ADDR); // Load the voltage of the pH board from the EEPROM
    float eepromPH2 = EEPROM_read(PH2ADDR); // Load the value of the pH board from the EEPROM
    float eepromVolt2 = EEPROM_read(Volt2ADDR); // Load the voltage of the pH board from the EEPROM
    
    EEPROM.commit();
    EEPROM.end();

    float PH_Cal[2][2] = {eepromPH1, eepromVolt1, eepromPH2, eepromVolt2};

    an.set_PH_Cal(PH_Cal);
}

void WaterQuality::setup()
{
    ADS1115_Setup(ADS1X15_ADDRESS1);
    ADS1115_Setup(ADS1X15_ADDRESS2);
    MCP23008_Setup(MCP23008_ADDRESS);
    PCA9685_Setup(PCA9685_I2C_ADDRESS);
    PH_Setup();
}
void WaterQuality::dump_config()
{
    LOG_I2C_DEVICE(this);
    if (this->is_failed())
        ESP_LOGE(TAG, "Communication failed!");
    else
        ESP_LOGI(TAG, "Communication Successfulled!");
    LOG_UPDATE_INTERVAL(this);
        
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    Wire.beginTransmission(TCA9548_ADDRESS);
    if(!Wire.endTransmission())
    {
        ESP_LOGI(TAG, "TCA9548:");

        for (uint8_t t=0; t<8; t++)
        {
            tcaselect(t);
            ESP_LOGI(TAG, "Channel %d:", t);

            for (uint8_t addr = 0; addr<=127; addr++) 
            {
                if (addr == TCA9548_ADDRESS) continue;

                Wire.beginTransmission(addr);
                if (!Wire.endTransmission()) 
                {
                    if (addr < 16)
                        ESP_LOGI(TAG, "Found I2C 0x0%x",addr);
                    else
                        ESP_LOGI(TAG, "Found I2C 0x%x",addr);
                }
                else if(Wire.endTransmission() == 4)
                    ESP_LOGE(TAG, "Found the same I2C 0x%x",addr);
            }
        }
    }
    else
    {
        ESP_LOGI(TAG, "I2C:");
        
        for (uint8_t addr = 0; addr<=127; addr++) 
        {
            Wire.beginTransmission(addr);
            if (!Wire.endTransmission()) 
            {
                if (addr < 16)
                    ESP_LOGI(TAG, "Found I2C 0x0%x",addr);
                else
                    ESP_LOGI(TAG, "Found I2C 0x%x",addr);
            }
            else if(Wire.endTransmission() == 4)
                ESP_LOGE(TAG, "Found the same I2C 0x%x",addr);
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ESP_LOGCONFIG(TAG, "PCA9685:");
    if (this->extclk_) {
        ESP_LOGCONFIG(TAG, "  EXTCLK: enabled");
    } else {
        ESP_LOGCONFIG(TAG, "  EXTCLK: disabled");
        ESP_LOGCONFIG(TAG, "  Frequency: %.0f Hz", this->frequency_);
    }
    if (this->is_failed()) {
        ESP_LOGE(TAG, "Setting up PCA9685 failed!");
    }
    
    uint8_t dose = 0, circ = 0;
    float* calib_gain = pump.get_Pump_Calibration_Gain();
    uint8_t* type = pump.get_Pump_Type();

    for (uint8_t i = 0; i < 6; i++)
        if (type[i] == 1)
            dose += 1;
        else if (type[i] == 2)
            circ += 1;

    ESP_LOGI(TAG, "Pump_dose = %d", dose);
    ESP_LOGI(TAG, "Pump_circ = %d", circ);
    for (uint8_t i = 0; i < 6; i++)
    {
        ESP_LOGI(TAG, "Pump_Calibration_Gain[%d] = %.2f", i, calib_gain[i]);
        ESP_LOGI(TAG, "Pump_Type[%d] = %d", i, type[i]);
    }

    uint16_t *resmin = an.get_ResMin(), *resmax = an.get_ResMax();
    for (uint8_t i = 0; i < sizeof(resmin) / sizeof(resmin[0]); i++)
        ESP_LOGI(TAG, "ResMin[%d] = %d, ResMax[%d] = %d", i, resmin[i], i, resmax[i]);
    ESP_LOGI(TAG, "EC_ch = %d, EC_type = %d", an.get_EC_Ch(), an.get_EC_Type());
    ESP_LOGI(TAG, "PH_ch = %d, PH_type = %d", an.get_PH_Ch(), an.get_PH_Type());
}
void WaterQuality::loop()
{
    pump.Calibration_Controller();
    sensor();
}
void WaterQuality::update()
{
    float a[8], p[16] = {0}, e[6] = {0};
    bool d[4];

    ADS1115_Driver(a);
    an.Analog_Input_Driver(a);

    dig.Digital_Output_Driver(d);
    MCP23008_Driver(d);
    dig.Digital_Input_Driver(d);

    pump.Generic_Pump_Driver(p);
    ser.Servo_Driver(p);
    PCA9685_Driver(p);
}

void WaterQuality::version(const uint8_t ver)
{
    an.set_version(ver);
}
void WaterQuality::pump_calibration_mode(std::vector<bool> &pcal)
{
    bool* pcal_ = pump.get_Pump_Calibration_Mode();
    std::vector<bool> pc(pcal_, pcal_ + 6);

    if (pc != pcal)
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            pcal_[i] = pcal[i];
            ESP_LOGD(TAG, "Pump_Calibration_Mode[%d] = %d", i, pcal_[i]);
        }
    }
}
void WaterQuality::pump_calibration_gain(const std::vector<float> &pcal)
{
    float pcal_[6];

    for (uint8_t i = 0; i < 6; i++)
        pcal_[i] = pcal[i];

    pump.set_Pump_Calibration_Gain(pcal_);
}
void WaterQuality::pump_type(const std::vector<uint8_t> &ptype)
{
    uint8_t ptype_[6];
    
    for (uint8_t i = 0; i < 6; i++)
        ptype_[i] = ptype[i];

    pump.set_Pump_Type(ptype_);   
}
void WaterQuality::pump_mode(std::vector<uint8_t> &pmode)
{
    uint8_t* pmode_ = pump.get_Pump_Mode();
    std::vector<uint8_t> pm(pmode_, pmode_ + 6);

    if (pm != pmode)
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            pmode_[i] = pmode[i];
            ESP_LOGD(TAG, "Pump_Mode[%d] = %d", i, pmode_[i]);
        }
    }
}
void WaterQuality::pump_dose(std::vector<float> &pdose)
{
    uint8_t* ptype = pump.get_Pump_Type();
    uint8_t* pmode = pump.get_Pump_Mode();
    uint8_t* pstat = pump.get_Pump_Status();
    float* pdose_ = pump.get_Pump_Dose();
    std::vector<float> pd(pdose_, pdose_ + 6);

    if (pd != pdose && !pump.get_Pump_Calibration_Mode_Check())
        for (uint8_t i = 0; i < 6; i++)
            if (ptype[i] == 1)
                if (pmode[i] == 0)
                {
                    if (!(pstat[i] == 1))
                    {
                        pdose_[i] = pdose[i];
                        ESP_LOGD(TAG, "Pump_Dose[%d] = %f", i, pdose_[i]);
                    }
                    // else
                    // {
                    //     pdose_[i] += pdose[i];
                    //     ESP_LOGD(TAG, "Pump_Dose[%d] = %f", i, pdose_[i]);
                    // }
                }
}
void WaterQuality::pump_circulation(std::vector<float> &pcirc)
{
    uint8_t* ptype = pump.get_Pump_Type();
    uint8_t* pmode = pump.get_Pump_Mode();
    uint8_t* pstat = pump.get_Pump_Status();
    float* pcirc_ = pump.get_Pump_Circulation();
    std::vector<float> pc(pcirc_, pcirc_ + 6);

    if (pc != pcirc && !pump.get_Pump_Calibration_Mode_Check())
        for (uint8_t i = 0; i < 6; i++)
            if (ptype[i] == 2)
                if (pmode[i] == 0)
                {
                    if (!(pstat[i] == 1))
                    {
                        pcirc_[i] = pcirc[i];
                        ESP_LOGD(TAG, "Pump_Circulation[%d] = %f", i, pcirc_[i]);
                    }
                    // else
                    // {
                    //     pcirc_[i] += pcirc[i];
                    //     ESP_LOGD(TAG, "Pump_Circulation[%d] = %f", i, pcirc_[i]);
                    // }
                }
}
void WaterQuality::pump_reset(std::vector<bool> &pres)
{
    bool* pres_ = pump.get_Pump_Reset();
    std::vector<bool> pr(pres_, pres_ + 6);

    if (pr != pres && !pump.get_Pump_Calibration_Mode_Check())
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            pres_[i] = pres[i];
            ESP_LOGD(TAG, "Pump_Reset[%d] = %d", i, pres_[i]);
        }
    }
}
void WaterQuality::servo_mode(std::vector<bool> &smode)
{
    bool* smode_ = ser.get_Servo_Mode();
    std::vector<bool> sm(smode_, smode_ + 8);
    
    if (sm != smode)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            smode_[i] = smode[i];
            ESP_LOGD(TAG, "Servo_Mode[%d] = %d", i, smode_[i]);
        }
    }
}
void WaterQuality::servo_position(std::vector<uint8_t> &spos)
{
    uint8_t* spos_ = ser.get_Servo_Position();
    std::vector<uint8_t> sp(spos_, spos_ + 8);
    
    if (sp != spos)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            spos_[i] = spos[i];
            ESP_LOGD(TAG, "Servo_Position[%d] = %d", i, spos_[i]);
        }
    }
}
void WaterQuality::level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax)
{    
    uint16_t rminArray[2] = {0}, rmaxArray[2] = {0};

    for (uint8_t i = 0; i < rmin.size(); i++)
    {
        rminArray[i] = rmin[i];
        rmaxArray[i] = rmax[i];
    }

    an.set_ResMin(rminArray);
    an.set_ResMax(rmaxArray);
}
void WaterQuality::ph_calibration(float ph)
{
    float voltage = an.phVoltage * 1000; // Convert from V to mV

    EEPROM.begin(EEPROM_SIZE);
    
    float eepromPH1 = EEPROM_read(PH1ADDR); // Load the value of the pH board from the EEPROM
    float eepromVolt1 = EEPROM_read(Volt1ADDR); // Load the voltage of the pH board from the EEPROM
    float eepromPH2 = EEPROM_read(PH2ADDR); // Load the value of the pH board from the EEPROM
    float eepromVolt2 = EEPROM_read(Volt2ADDR); // Load the voltage of the pH board from the EEPROM
    
    if (round(ph) != eepromPH2)
    {
        eepromPH1 = ph;
        eepromVolt1 = voltage;
        EEPROM_write(PH1ADDR, ph); // Store the current pH value as
        EEPROM_write(Volt1ADDR, voltage); // Store the current pH voltage as
        ESP_LOGI(TAG,"Calibrated to pH = %f", ph);
    }
    else if (round(ph) != eepromPH1)
    {
        eepromPH2 = ph;
        eepromVolt2 = voltage;
        EEPROM_write(PH2ADDR, ph); // Store the current pH value as
        EEPROM_write(Volt2ADDR, voltage); // Store the current pH voltage as
        ESP_LOGI(TAG,"Calibrated to pH = %f", ph);
    }
    
    EEPROM.commit();
    EEPROM.end();

    ESP_LOGI(TAG,"PH1ADDR = %d    eepromPH1 = %f", PH1ADDR, eepromPH1);
    ESP_LOGI(TAG,"Volt1ADDR = %d    eepromVolt1 = %f", Volt1ADDR, eepromVolt1);
    ESP_LOGI(TAG,"PH2ADDR = %d    eepromPH2 = %f", PH2ADDR, eepromPH2);
    ESP_LOGI(TAG,"Volt2ADDR = %d    eepromVolt2 = %f", Volt2ADDR, eepromVolt2);

    float PH_Cal[2][2] = {eepromPH1, eepromVolt1, eepromPH2, eepromVolt2};

    an.set_PH_Cal(PH_Cal);
}
void WaterQuality::ph(const uint8_t ch, const uint8_t type)
{
    an.set_PH_Ch(ch);
    an.set_PH_Type(type);
}
void WaterQuality::ec_calibration(float cal)
{
    an.set_EC_Cal(cal);
}
void WaterQuality::ec(const uint8_t ch, const uint8_t type)
{
    an.set_EC_Ch(ch);
    an.set_EC_Type(type);
}
void WaterQuality::digital_out(std::vector<bool> &dout)
{
    bool* dout_ = dig.get_Digital_Output();
    std::vector<bool> d(dout_, dout_ + 4);

    if (d != dout)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            dout_[i] = dout[i];
            ESP_LOGD(TAG, "DigOut_Status[%d] = %d", i, dout_[i]);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor
void WaterQuality::sensor()
{
    if (this->Pump_Tot_ != nullptr)
    {
        uint32_t (*ptot)[2] = pump.get_Pump_Total();
        std::stringstream pt;

        for (uint8_t i = 0; i < 6; i++)
            if (i == 0)
                pt << std::fixed << std::setprecision(4) << ptot[i][0] + ptot[i][1] / 10000000.0;
            else
                pt << "," << std::fixed << std::setprecision(4) << ptot[i][0] + ptot[i][1] / 10000000.0;
            
        this->Pump_Tot_->publish_state(pt.str());
    }
    if (this->Pump_Stat_ != nullptr)
    { 
        uint8_t* pstat = pump.get_Pump_Status();
        std::stringstream ps;

        for (uint8_t i = 0; i < 6; i++)
            if (i == 0)
                ps << std::fixed << std::setprecision(0) << static_cast<int>(pstat[i]);
            else
                ps << "," << std::fixed << std::setprecision(0) << static_cast<int>(pstat[i]);
            
        this->Pump_Stat_->publish_state(ps.str());
    }
    if (this->Servo_Stat_ != nullptr)
    { 
        bool* sstat = ser.get_Servo_Status();
        std::stringstream ss;

        for (uint8_t i = 0; i < 8; i++)
            if (i == 0)
                ss << std::fixed << std::setprecision(0) << static_cast<int>(sstat[i]);
            else
                ss << "," << std::fixed << std::setprecision(0) << static_cast<int>(sstat[i]);
            
        this->Servo_Stat_->publish_state(ss.str());
    }
    if (this->AnInWTemp_Val_ != nullptr)    { this->AnInWTemp_Val_->publish_state(an.get_WatTemp_Val()); }
    if (this->AnInVPow_Val_ != nullptr)     { this->AnInVPow_Val_->publish_state(an.get_VoltPow_Val()); }
    if (this->AnInLvl_Perc_ != nullptr) 
    {
        float* lvl = an.get_Lvl_Perc();
        std::stringstream ap;

        for (uint8_t i = 0; i < 2; i++)
            if (i == 0)
                ap << std::fixed << std::setprecision(2) << lvl[i];
            else
                ap << "," << std::fixed << std::setprecision(2) << lvl[i];

        this->AnInLvl_Perc_->publish_state(ap.str());
    }
    if (this->AnInEC_Val_ != nullptr)       { this->AnInEC_Val_->publish_state(an.get_EC_Val()); }
    if (this->AnInPH_Val_ != nullptr)       { this->AnInPH_Val_->publish_state(an.get_PH_Val()); }
    if (this->AnInGen_Val_ != nullptr) 
    {
        float* gen = an.get_Gen_Val();
        std::stringstream av;

        for (uint8_t i = 0; i < 2; i++)
            if (i == 0)
                av << std::fixed << std::setprecision(2) << gen[i];
            else
                av << "," << std::fixed << std::setprecision(2) << gen[i];
    
        this->AnInGen_Val_->publish_state(av.str());
    }
    if (this->DigIn_Stat_ != nullptr) 
    {
        bool* din = dig.get_Digital_Input();
        std::stringstream ds;

        for (uint8_t i = 0; i < 4; i++)
            if (i == 0)
                ds << std::fixed << std::setprecision(0) << static_cast<int>(din[i]);
            else
                ds << "," << std::fixed << std::setprecision(0) << static_cast<int>(din[i]);

        this->DigIn_Stat_->publish_state(ds.str());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace water_quality
}  // namespace esphome