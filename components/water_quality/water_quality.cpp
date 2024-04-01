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

void WaterQuality::setup()
{
    ADS1115_Setup(ADS1X15_ADDRESS1);
    ADS1115_Setup(ADS1X15_ADDRESS2);
    MCP23008_Setup(MCP23008_ADDRESS);
    PCA9685_Setup(PCA9685_I2C_ADDRESS);
    pump.Calibration_Status();
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
        ESP_LOGCONFIG(TAG, "TCA9548:");

        for (size_t t=0; t<8; t++)
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
    float* calib = pump.get_Pump_Calib_Gain();
    uint8_t* type = pump.get_Pump_Type();
    uint8_t* model = pump.get_Pump_Model();

    if (pump.get_Calibration_Mode())
        ESP_LOGI(TAG, "Pump Calibration Enable");

    for (size_t i = 0; i < 6; i++)
        if (type[i] == 1)
            dose += 1;
        else if (type[i] == 2)
            circ += 1;

    ESP_LOGI(TAG, "Pump_dose = %d", dose);
    ESP_LOGI(TAG, "Pump_circ = %d", circ);
    for (size_t i = 0; i < 6; i++)
    {
        ESP_LOGI(TAG, "Pump_Calib_Gain[%d] = %.2f", i, calib[i]);
        ESP_LOGI(TAG, "Pump_Type[%d] = %d", i, type[i]);
        ESP_LOGI(TAG, "Pump_Model[%d] = %d", i, model[i]);
    }

    uint16_t *resmin = an.get_ResMin(), *resmax = an.get_ResMax();
    for (size_t i = 0; i < sizeof(resmin) / sizeof(resmin[0]); i++)
        ESP_LOGI(TAG, "ResMin[%d] = %d, ResMax[%d] = %d", i, resmin[i], i, resmax[i]);
    ESP_LOGI(TAG, "EC_ch = %d, EC_type = %d", an.get_EC_Ch(), an.get_EC_Type());
    ESP_LOGI(TAG, "PH_ch = %d, PH_type = %d", an.get_PH_Ch(), an.get_PH_Type());
}
void WaterQuality::loop() 
{
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
    ser.Servo_driver(p);
    PCA9685_Driver(p);

    pump.Serial_Com_Pump_Driver(e);
    EZOPMP_Driver(e);

    sensor();
}

void WaterQuality::version(const uint8_t ver)
{
    an.set_version(ver);
}
void WaterQuality::pump_calibration(std::vector<bool> &pcal)
{
    bool* pcal_ = pump.get_Pump_Calibration();
    std::vector<bool> pc(pcal_, pcal_ + 6);

    if (pc != pres)
    {
        for (size_t i = 0; i < 6; i++)
        {
            pcal_[i] = pcal[i];
            ESP_LOGD(TAG, "Pump_Calibration[%d] = %d", i, pcal_[i]);
        }
    }
}
void WaterQuality::pump_calibration_gain(const std::vector<float> &pcal)
{
    float pcal_[6];

    for (size_t i = 0; i < 6; i++)
    {
        if (pcal[i] > 0)
            pcal_[i] = pcal[i] / 60;
        else
            pcal_[i] = 0;
    }

    pump.set_Pump_Calib_Gain(pcal_);
}
void WaterQuality::pump_type(const std::vector<uint8_t> &ptype)
{
    uint8_t ptype_[6];
    
    for (size_t i = 0; i < 6; i++)
        ptype_[i] = ptype[i];

    pump.set_Pump_Type(ptype_);   
}
void WaterQuality::pump_model(const std::vector<uint8_t> &pmodel)
{
    uint8_t pmodel_[6];
    
    for (size_t i = 0; i < 6; i++)
        pmodel_[i] = pmodel[i];

    pump.set_Pump_Model(pmodel_);   
}
void WaterQuality::pump_mode(std::vector<uint8_t> &pmode)
{
    uint8_t* pmode_ = pump.get_Pump_Mode();
    std::vector<uint8_t> pm(pmode_, pmode_ + 6);

    if (pm != pmode && !pump.get_Calibration_Mode())
    {
        for (size_t i = 0; i < 6; i++)
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

    if (pd != pdose && !pump.get_Calibration_Mode())
        for (size_t i = 0; i < 6; i++)
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

    if (pc != pcirc && !pump.get_Calibration_Mode())
        for (size_t i = 0; i < 6; i++)
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

    if (pr != pres && !pump.get_Calibration_Mode())
    {
        for (size_t i = 0; i < 6; i++)
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
        for (size_t i = 0; i < 8; i++)
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
        for (size_t i = 0; i < 8; i++)
        {
            spos_[i] = spos[i];
            ESP_LOGD(TAG, "Servo_Position[%d] = %d", i, spos_[i]);
        }
    }
}
void WaterQuality::level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax)
{    
    uint16_t rminArray[2] = {0}, rmaxArray[2] = {0};

    for (size_t i = 0; i < rmin.size(); i++)
    {
        rminArray[i] = rmin[i];
        rmaxArray[i] = rmax[i];
    }

    an.set_ResMin(rminArray);
    an.set_ResMax(rmaxArray);
}
void WaterQuality::ec(const uint8_t ch, const uint8_t type)
{
    an.set_EC_Ch(ch);
    an.set_EC_Type(type);
}
void WaterQuality::ph(const uint8_t ch, const uint8_t type)
{
    an.set_PH_Ch(ch);
    an.set_PH_Type(type);
}
void WaterQuality::digital_out(std::vector<bool> &dout)
{
    bool* dout_ = dig.get_Digital_Out();
    std::vector<bool> d(dout_, dout_ + 4);

    if (d != dout)
    {
        for (size_t i = 0; i < 4; i++)
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

        for (size_t i = 0; i < 6; i++)
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

        for (size_t i = 0; i < 6; i++)
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

        for (size_t i = 0; i < 8; i++)
            if (i == 0)
                ss << std::fixed << std::setprecision(0) << static_cast<int>(sstat[i]);
            else
                ss << "," << std::fixed << std::setprecision(0) << static_cast<int>(sstat[i]);
            
        this->Servo_Stat_->publish_state(ss.str());
    }
    if (this->AnInWTemp_Val_ != nullptr)    { this->AnInWTemp_Val_->publish_state(an.get_WTemp_Val()); }
    if (this->AnInVPow_Val_ != nullptr)     { this->AnInVPow_Val_->publish_state(an.get_VPow_Val()); }
    if (this->AnInLvl_Perc_ != nullptr) 
    {
        float* lvl = an.get_Lvl_Perc();
        std::stringstream ap;

        for (size_t i = 0; i < 2; i++)
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

        for (size_t i = 0; i < 2; i++)
            if (i == 0)
                av << std::fixed << std::setprecision(2) << gen[i];
            else
                av << "," << std::fixed << std::setprecision(2) << gen[i];
    
        this->AnInGen_Val_->publish_state(av.str());
    }
    if (this->DigIn_Stat_ != nullptr) 
    {
        bool* din = dig.get_Digital_In();
        std::stringstream ds;

        for (size_t i = 0; i < 4; i++)
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