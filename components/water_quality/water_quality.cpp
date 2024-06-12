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
    float acidPh = 4.0;
    float neutralPh = 7.0;
    float basePh = 10.0;

    float acidVoltage = 0; // V
    float neutralVoltage = 0; // V
    float baseVoltage = 0; // V

    uint8_t version = an.get_PH_Type();
    if (version == 1) // V1
    {
        acidVoltage = 0.41; // V
        neutralVoltage = 1.78; // V
        baseVoltage = 3.02; // V
    }
    else if (version == 2) // V2
    {
        acidVoltage = 2.03; // V
        neutralVoltage = 1.50; // V
        baseVoltage = 0.97; // V
    }

    if (EEPROM.read(PH1_VAL_ADDR) == 0xFF && EEPROM.read(PH1_VAL_ADDR + 1) == 0xFF && EEPROM.read(PH1_VAL_ADDR + 2) == 0xFF && EEPROM.read(PH1_VAL_ADDR + 3))
        EEPROM_write(PH1_VAL_ADDR, neutralPh); // New EEPROM, write typical pH value
    if (EEPROM.read(PH1_VOLT_ADDR) == 0xFF && EEPROM.read(PH1_VOLT_ADDR + 1) == 0xFF && EEPROM.read(PH1_VOLT_ADDR + 2) == 0xFF && EEPROM.read(PH1_VOLT_ADDR + 3))
        EEPROM_write(PH1_VOLT_ADDR, neutralVoltage); // New EEPROM, write typical pH voltage
    if (EEPROM.read(PH2_VAL_ADDR) == 0xFF && EEPROM.read(PH2_VAL_ADDR + 1) == 0xFF && EEPROM.read(PH2_VAL_ADDR + 2) == 0xFF && EEPROM.read(PH2_VAL_ADDR + 3))
        EEPROM_write(PH2_VAL_ADDR, acidPh); // New EEPROM, write typical pH value
    if (EEPROM.read(PH2_VOLT_ADDR) == 0xFF && EEPROM.read(PH2_VOLT_ADDR + 1) == 0xFF && EEPROM.read(PH2_VOLT_ADDR + 2) == 0xFF && EEPROM.read(PH2_VOLT_ADDR + 3))
        EEPROM_write(PH2_VOLT_ADDR, acidVoltage); // New EEPROM, write typical pH voltage
    
    float eepromPH1val = EEPROM_read(PH1_VAL_ADDR); // Load the value of the pH board from the EEPROM
    float eepromPH1volt = EEPROM_read(PH1_VOLT_ADDR); // Load the voltage of the pH board from the EEPROM
    float eepromPH2val = EEPROM_read(PH2_VAL_ADDR); // Load the value of the pH board from the EEPROM
    float eepromPH2volt = EEPROM_read(PH2_VOLT_ADDR); // Load the voltage of the pH board from the EEPROM

    ESP_LOGD(TAG,"PH1_VAL_ADDR = %d    eepromPH1val = %f", PH1_VAL_ADDR, eepromPH1val);
    ESP_LOGD(TAG,"PH1_VOLT_ADDR = %d    eepromPH1volt = %f", PH1_VOLT_ADDR, eepromPH1volt);
    ESP_LOGD(TAG,"PH2_VAL_ADDR = %d    eepromPH2val = %f", PH2_VAL_ADDR, eepromPH2val);
    ESP_LOGD(TAG,"PH2_VOLT_ADDR = %d    eepromPH2volt = %f", PH2_VOLT_ADDR, eepromPH2volt);

    float PH_Cal[2][2] = {eepromPH1val, eepromPH1volt, eepromPH2val, eepromPH2volt};

    an.set_PH_Cal(PH_Cal);
}
void PH_Clear()
{
    for (uint8_t i = PH1_VAL_ADDR; i < PH2_VOLT_ADDR; i++)
        EEPROM.write(i, 0xFF);

    EEPROM.commit();

    ESP_LOGD(TAG, "PH Clear");
    PH_Setup();
}
void EC_Setup()
{
    float kvalueLow = 1.0;
    float kvalueHigh = 1.0;

    float kvoltLow = 1.0;
    float kvoltHigh = 1.0;

    if (EEPROM.read(EC1_VAL_ADDR) == 0xFF && EEPROM.read(EC1_VAL_ADDR + 1) == 0xFF && EEPROM.read(EC1_VAL_ADDR + 2) == 0xFF && EEPROM.read(EC1_VAL_ADDR + 3))
        EEPROM_write(EC1_VAL_ADDR, kvalueLow); // New EEPROM, write typical EC value
    if (EEPROM.read(EC1_VOLT_ADDR) == 0xFF && EEPROM.read(EC1_VOLT_ADDR + 1) == 0xFF && EEPROM.read(EC1_VOLT_ADDR + 2) == 0xFF && EEPROM.read(EC1_VOLT_ADDR + 3))
        EEPROM_write(EC1_VOLT_ADDR, kvoltLow); // New EEPROM, write typical EC voltage
    if (EEPROM.read(EC2_VAL_ADDR) == 0xFF && EEPROM.read(EC2_VAL_ADDR + 1) == 0xFF && EEPROM.read(EC2_VAL_ADDR + 2) == 0xFF && EEPROM.read(EC2_VAL_ADDR + 3))
        EEPROM_write(EC2_VAL_ADDR, kvalueHigh); // New EEPROM, write typical EC value
    if (EEPROM.read(EC2_VOLT_ADDR) == 0xFF && EEPROM.read(EC2_VOLT_ADDR + 1) == 0xFF && EEPROM.read(EC2_VOLT_ADDR + 2) == 0xFF && EEPROM.read(EC2_VOLT_ADDR + 3))
        EEPROM_write(EC2_VOLT_ADDR, kvoltHigh); // New EEPROM, write typical EC voltage
    
    float eepromEC1val = EEPROM_read(EC1_VAL_ADDR); // Load the value of the EC board from the EEPROM
    float eepromEC1volt = EEPROM_read(EC1_VOLT_ADDR); // Load the voltage of the EC board from the EEPROM
    float eepromEC2val = EEPROM_read(EC2_VAL_ADDR); // Load the value of the EC board from the EEPROM
    float eepromEC2volt = EEPROM_read(EC2_VOLT_ADDR); // Load the voltage of the EC board from the EEPROM

    ESP_LOGD(TAG,"EC1_VAL_ADDR = %d    eepromEC1val = %f", EC1_VAL_ADDR, eepromEC1val);
    ESP_LOGD(TAG,"EC1_VOLT_ADDR = %d    eepromEC1volt = %f", EC1_VOLT_ADDR, eepromEC1volt);
    ESP_LOGD(TAG,"EC2_VAL_ADDR = %d    eepromEC2val = %f", EC2_VAL_ADDR, eepromEC2val);
    ESP_LOGD(TAG,"EC2_VOLT_ADDR = %d    eepromEC2volt = %f", EC2_VOLT_ADDR, eepromEC2volt);

    float EC_Cal[2][2] = {eepromEC1val, eepromEC1volt, eepromEC2val, eepromEC2volt};

    an.set_EC_Cal(EC_Cal);
}
void EC_Clear()
{
    for (uint8_t i = EC1_VAL_ADDR; i < EC2_VOLT_ADDR; i++)
        EEPROM.write(i, 0xFF);

    EEPROM.commit();

    ESP_LOGD(TAG, "EC Clear");
    EC_Setup();
}

void WaterQuality::setup()
{
    EEPROM.begin(EEPROM_SIZE);
    ADS1115_Setup(ADS1X15_ADDRESS1);
    ADS1115_Setup(ADS1X15_ADDRESS2);
    MCP23008_Setup(MCP23008_ADDRESS);
    PCA9685_Setup(PCA9685_I2C_ADDRESS);
    PH_Setup();
    EC_Setup();
}
    static uint32_t time = millis();
    static uint16_t sample = 0;
void WaterQuality::dump_config()
{
    EC_Setup();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  I2C

    ESP_LOGCONFIG(TAG, "");
    ESP_LOGCONFIG(TAG, "I2C:");
    
    LOG_I2C_DEVICE(this);
    LOG_UPDATE_INTERVAL(this);
    if (this->is_failed())
        ESP_LOGE(TAG, "  Communication failed!");
    else
        ESP_LOGI(TAG, "  Communication Successfulled!");
        
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    Wire.beginTransmission(TCA9548_ADDRESS);
    if(!Wire.endTransmission())
    {
        ESP_LOGCONFIG(TAG, "  TCA9548:");

        for (uint8_t t=0; t<8; t++)
        {
            tcaselect(t);
            ESP_LOGI(TAG, "    Channel %d:", t);

            for (uint8_t addr = 0; addr<=127; addr++) 
            {
                if (addr == TCA9548_ADDRESS) continue;

                Wire.beginTransmission(addr);
                if (!Wire.endTransmission()) 
                {
                    if (addr < 16)
                        ESP_LOGI(TAG, "      Found I2C 0x0%x",addr);
                    else
                        ESP_LOGI(TAG, "      Found I2C 0x%x",addr);
                }
                else if(Wire.endTransmission() == 4)
                    ESP_LOGE(TAG, "      Found the same I2C 0x%x",addr);
            }
        }
    }
    else
    {
        ESP_LOGCONFIG(TAG, "  I2C Devices:");

        for (uint8_t addr = 0; addr<=127; addr++) 
        {
            Wire.beginTransmission(addr);
            if (!Wire.endTransmission()) 
            {
                if (addr < 16)
                    ESP_LOGI(TAG, "    Found I2C 0x0%x",addr);
                else
                    ESP_LOGI(TAG, "    Found I2C 0x%x",addr);
            }
            else if(Wire.endTransmission() == 4)
                ESP_LOGE(TAG, "    Found the same I2C 0x%x",addr);
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685

    ESP_LOGCONFIG(TAG, "PCA9685:");
    if (this->extclk_) {
        ESP_LOGI(TAG, "  EXTCLK: enabled");
    } else {
        ESP_LOGI(TAG, "  EXTCLK: disabled");
        ESP_LOGI(TAG, "  Frequency: %.0f Hz", this->frequency_);
    }
    if (this->is_failed()) {
        ESP_LOGE(TAG, "  Setting up PCA9685 failed!");
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Pumps

    ESP_LOGCONFIG(TAG, "Pumps:");
    uint8_t dose = 0, circ = 0;
    float* calib_gain = pump.get_Pump_Calibration_Gain();
    uint8_t* type = pump.get_Pump_Type();

    for (uint8_t i = 0; i < 6; i++)
        if (type[i] == 1)
            dose += 1;
        else if (type[i] == 2)
            circ += 1;

    ESP_LOGI(TAG, "  Pump dose quantity: %d", dose);
    ESP_LOGI(TAG, "  Pump circ quantity: %d", circ);
    for (uint8_t i = 0; i < 6; i++)
    {
        ESP_LOGCONFIG(TAG, "  Pump%d:", i + 1);
        ESP_LOGI(TAG, "    Pump Calibration Gain = %.2f", calib_gain[i]);
        ESP_LOGI(TAG, "    Pump Type = %d", type[i]);
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Level

    ESP_LOGCONFIG(TAG, "Level:");
    uint16_t *resmin = an.get_ResMin(), *resmax = an.get_ResMax();
    for (uint8_t i = 0; i < sizeof(resmin) / sizeof(resmin[0]); i++)
        ESP_LOGI(TAG, "  ResMin%d = %d, ResMax%d = %d", i + 1, resmin[i], i + 1, resmax[i]);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PH

    ESP_LOGCONFIG(TAG, "PH:");
    ESP_LOGI(TAG, "  PH_ch = %d, PH_type = %d", an.get_PH_Ch(), an.get_PH_Type());

    float eepromPH1val = EEPROM_read(PH1_VAL_ADDR); // Load the value of the pH board from the EEPROM
    float eepromPH1volt = EEPROM_read(PH1_VOLT_ADDR); // Load the voltage of the pH board from the EEPROM
    float eepromPH2val = EEPROM_read(PH2_VAL_ADDR); // Load the value of the pH board from the EEPROM
    float eepromPH2volt = EEPROM_read(PH2_VOLT_ADDR); // Load the voltage of the pH board from the EEPROM

    ESP_LOGI(TAG,"  eepromPH1val = %f", eepromPH1val);
    ESP_LOGI(TAG,"  eepromPH1volt = %f", eepromPH1volt);
    ESP_LOGI(TAG,"  eepromPH2val = %f", eepromPH2val);
    ESP_LOGI(TAG,"  eepromPH2volt = %f", eepromPH2volt);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  EC

    ESP_LOGCONFIG(TAG, "EC:");
    ESP_LOGI(TAG, "  EC_ch = %d, EC_type = %d", an.get_EC_Ch(), an.get_EC_Type());

    float eepromEC1val = EEPROM_read(EC1_VAL_ADDR); // Load the value of the EC board from the EEPROM
    float eepromEC1volt = EEPROM_read(EC1_VOLT_ADDR); // Load the voltage of the EC board from the EEPROM
    float eepromEC2val = EEPROM_read(EC2_VAL_ADDR); // Load the value of the EC board from the EEPROM
    float eepromEC2volt = EEPROM_read(EC2_VOLT_ADDR); // Load the voltage of the EC board from the EEPROM

    ESP_LOGI(TAG,"  Calibrated EC val = %f", eepromEC1val);
    ESP_LOGI(TAG,"  Calibrated EC volt = %f", eepromEC1volt);
    ESP_LOGI(TAG,"  Calibrated EC val = %f", eepromEC2val);
    ESP_LOGI(TAG,"  Calibrated EC volt = %f", eepromEC2volt);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
void WaterQuality::loop()
{
    pump.Calibration_Controller();
    sensor();

    if (millis() - time >= 1000)
    {
        time = millis();
        ESP_LOGI(TAG, "sample = %d", sample);
        sample = 0;
    }

    sample++;
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
    if (ph > 0)
    {
        float voltage = an.phVoltage;
        float eepromPH1val = 7, eepromPHvolt1, eepromPH2val = 4, eepromPHvolt2;
        static float eepromPH1val_backup = eepromPH1val, eepromPH2val_backup = eepromPH2val;
        
        eepromPH1val = EEPROM_read(PH1_VAL_ADDR); // Load the value of the pH board from the EEPROM
        eepromPHvolt1 = EEPROM_read(PH1_VOLT_ADDR); // Load the voltage of the pH board from the EEPROM
        eepromPH2val = EEPROM_read(PH2_VAL_ADDR); // Load the value of the pH board from the EEPROM
        eepromPHvolt2 = EEPROM_read(PH2_VOLT_ADDR); // Load the voltage of the pH board from the EEPROM
    
        static bool q = 0;
        if (round(ph) != round(eepromPH2val) && round(ph) != round(eepromPH2val_backup) && !q || round(ph) == round(eepromPH1val))
        {
            q = 1;
            eepromPH1val = ph;
            eepromPHvolt1 = voltage;
            EEPROM_write(PH1_VAL_ADDR, ph); // Store the current pH value as
            EEPROM_write(PH1_VOLT_ADDR, voltage); // Store the current pH voltage as
            ESP_LOGI(TAG,"Calibrated to pH = %f", ph);
        }
        else if (round(ph) != round(eepromPH1val) && round(ph) != round(eepromPH1val_backup) && q || round(ph) == round(eepromPH2val))
        {
            q = 0;
            eepromPH2val = ph;
            eepromPHvolt2 = voltage;
            EEPROM_write(PH2_VAL_ADDR, ph); // Store the current pH value as
            EEPROM_write(PH2_VOLT_ADDR, voltage); // Store the current pH voltage as
            ESP_LOGI(TAG,"Calibrated to pH = %f", ph);
        }
        
        EEPROM.commit();

        ESP_LOGD(TAG,"PH1_VAL_ADDR = %d    eepromPH1val = %f", PH1_VAL_ADDR, eepromPH1val);
        ESP_LOGD(TAG,"PH1_VOLT_ADDR = %d    eepromPHvolt1 = %f", PH1_VOLT_ADDR, eepromPHvolt1);
        ESP_LOGD(TAG,"PH2_VAL_ADDR = %d    eepromPH2val = %f", PH2_VAL_ADDR, eepromPH2val);
        ESP_LOGD(TAG,"PH2_VOLT_ADDR = %d    eepromPHvolt2 = %f", PH2_VOLT_ADDR, eepromPHvolt2);

        float PH_Cal[2][2] = {eepromPH1val, eepromPHvolt1, eepromPH2val, eepromPHvolt2};

        an.set_PH_Cal(PH_Cal);
    }
    else
        PH_Clear();
}
void WaterQuality::ph(const uint8_t ch, const uint8_t type)
{
    an.set_PH_Ch(ch);
    an.set_PH_Type(type);
}
void WaterQuality::ec_calibration(float ec)
{
    float voltage = an.ecVoltage;
    float temperature = an.get_WatTemp_Val();
    
    float RES2 = 820.0;
    float ECREF = 200.0;

    float kvalueLow = EEPROM_read(EC1_VAL_ADDR);
    float kvalueHigh = EEPROM_read(EC2_VAL_ADDR);
    float kvalue =  kvalueLow; // set default K value: K = kvalueLow

    float rawEC = 1000 * voltage / RES2 / ECREF;
    float valueTemp = rawEC * kvalue;
    //automatic shift process
    //First Range:(0,2); Second Range:(2,20)
    if (valueTemp > 2.5)
        kvalue = kvalueHigh;
    else if (valueTemp < 2.0)
        kvalue = kvalueLow;

    float ecvalue = rawEC * kvalue; //calculate the EC value after automatic shift
    ecvalue /= (1.0 + 0.0185 * (temperature - 25.0)); //temperature compensation

    
    ec *= (1.0 + 0.0185 * (temperature - 25.0)); //temperature compensation
    if (ec > 0)
    {
        float voltage = an.ecVoltage;
        float eepromEC1val = 1.413, eepromEC1volt, eepromEC2val = 12.88, eepromEC2volt;
        static float eepromEC1val_backup = eepromEC1val, eepromEC2val_backup = eepromEC2val;
        
        eepromEC1val = EEPROM_read(EC1_VAL_ADDR); // Load the value of the EC board from the EEPROM
        eepromEC1volt = EEPROM_read(EC1_VOLT_ADDR); // Load the voltage of the EC board from the EEPROM
        eepromEC2val = EEPROM_read(EC2_VAL_ADDR); // Load the value of the EC board from the EEPROM
        eepromEC2volt = EEPROM_read(EC2_VOLT_ADDR); // Load the voltage of the EC board from the EEPROM

        static bool q = 0;
        if (round(ec) != round(eepromEC2val) && round(ec) != round(eepromEC2val_backup) && !q || round(ec) == round(eepromEC1val))
        {
            q = 1;
            eepromEC1val = ec;
            eepromEC1volt = voltage;
            EEPROM_write(EC1_VAL_ADDR, ec); // Store the current EC value as
            EEPROM_write(EC1_VOLT_ADDR, voltage); // Store the current EC voltage as
            ESP_LOGI(TAG,"Calibrated to EC = %f", ec);
        }
        else if (round(ec) != round(eepromEC1val) && round(ec) != round(eepromEC1val_backup) && q || round(ec) == round(eepromEC2val))
        {
            q = 0;
            eepromEC2val = ec;
            eepromEC2volt = voltage;
            EEPROM_write(EC2_VAL_ADDR, ec); // Store the current EC value as
            EEPROM_write(EC2_VOLT_ADDR, voltage); // Store the current EC voltage as
            ESP_LOGI(TAG,"Calibrated to EC = %f", ec);
        }
        
        EEPROM.commit();

    ESP_LOGD(TAG,"EC1_VAL_ADDR = %d    eepromEC1val = %f", EC1_VAL_ADDR, eepromEC1val);
    ESP_LOGD(TAG,"EC1_VOLT_ADDR = %d    eepromEC1volt = %f", EC1_VOLT_ADDR, eepromEC1volt);
    ESP_LOGD(TAG,"EC2_VAL_ADDR = %d    eepromEC2val = %f", EC2_VAL_ADDR, eepromEC2val);
    ESP_LOGD(TAG,"EC2_VOLT_ADDR = %d    eepromEC2volt = %f", EC2_VOLT_ADDR, eepromEC2volt);

        float EC_Cal[2][2] = {eepromEC1val, eepromEC1volt, eepromEC2val, eepromEC2volt};

        an.set_EC_Cal(EC_Cal);
    }
    else
        EC_Clear();
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