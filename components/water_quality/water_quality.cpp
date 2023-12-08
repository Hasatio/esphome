#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_analog.h"

namespace esphome {
namespace water_quality {

    Analog an;
    Digital dig;
    Pump pump;
    Servo ser;
    
static unsigned long timepoint = millis();

void WaterQuality::setup()
{
    
    ADS1115_Setup(ADS1X15_ADDRESS1);
    ADS1115_Setup(ADS1X15_ADDRESS2);
    // MCP23008_Setup();

    // PCA9685_Setup();
    
}
void WaterQuality::dump_config()
{
    LOG_I2C_DEVICE(this);
    if (this->is_failed())
        ESP_LOGE(TAG, "Communication failed!");
    else
        ESP_LOGI(TAG, "Communication Successfulled!");

LOG_I2C_DEVICE(this);
if (this->is_failed()) {
ESP_LOGE(TAG, "Communication with ADS1115 failed!");
}

    float f[8];
    ADS1115_Driver(f);
ESP_LOGI(TAG, "ads1: %f", f[0]);
ESP_LOGI(TAG, "ads2: %f", f[1]);
ESP_LOGI(TAG, "ads3: %f", f[2]);
ESP_LOGI(TAG, "ads4: %f", f[3]);
ESP_LOGI(TAG, "ads5: %f", f[4]);
ESP_LOGI(TAG, "ads6: %f", f[5]);
ESP_LOGI(TAG, "ads7: %f", f[6]);
ESP_LOGI(TAG, "ads8: %f\n", f[7]);
    an.Analog_Input_Driver(f);

ESP_LOGI(TAG, "WT: %f", an.get_WT_Val());
ESP_LOGI(TAG, "VPow: %f", an.get_VPow_Val());
float* lvl = an.get_Lvl_Perc();
ESP_LOGI(TAG, "Lvl1: %f", lvl[0]);
ESP_LOGI(TAG, "Lvl2: %f", lvl[1]);
ESP_LOGI(TAG, "EC: %f", an.get_EC_Val());
ESP_LOGI(TAG, "PH: %f", an.get_PH_Val());
float* gen = an.get_Gen_Val();
ESP_LOGI(TAG, "Gen1: %f", gen[0]);
ESP_LOGI(TAG, "Gen2: %f", gen[1]);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    // Wire.begin(SDA,SCL,frq);

    // for (size_t t=0; t<8; t++) 
    // {
    //   tcaselect(t);
    //   ESP_LOGI(TAG,"TCA Port %d", t);

    //   for (uint8_t addr = 0; addr<=127; addr++) 
    //   {
    //     if (addr == TCA9548_ADDRESS) continue;

    //     Wire.beginTransmission(addr);
    //     if (!Wire.endTransmission()) 
    //     {
    //       ESP_LOGI(TAG,"Found I2C 0x%x",addr);
    //     }
    //   }
    // }
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

    uint16_t resmin[] = an.get_ResMin(), resmax[] = an.get_ResMax();
    for (size_t i = 0; i < resmin.size(); i++)
    {
        ESP_LOGI(TAG,"ResMin[%d] = %d", i, resmin[i]);
        ESP_LOGI(TAG,"ResMax[%d] = %d", i, resmax[i]);
    }

    ESP_LOGI(TAG,"EC_ch = %d", an.get_EC_Ch());
    ESP_LOGI(TAG,"EC_type = %d", an.get_EC_Type());
    ESP_LOGI(TAG,"PH_ch = %d", an.get_PH_Ch());
    ESP_LOGI(TAG,"PH_type = %d", an.get_PH_Type());
}
void WaterQuality::loop() 
{
    // delay(1000);
}
void WaterQuality::update()
{
    float f[8];
    ADS1115_Driver(f);
    an.Analog_Input_Driver(f);
    // MCP23008_Driver();
    // pca9685();
    // pump_total();
    sensor();
    // an.set_WT_Val(1.23);
    // ESP_LOGD(TAG,"test = %f", request_measurement());
    // ESP_LOGD(TAG,"vpow test = %f", an.get_VPow_Val());
    
}

    bool pd[6], pc[6];

void WaterQuality::pump_calib_gain(const std::vector<float> &pcg)
{
    pump.Pump_Calib_Gain = pcg;
}
void WaterQuality::pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
    pump.dose = d;
    pump.circ = c;
    
    pump.Pump_Type = ptype;
}
void WaterQuality::pump_dose(std::vector<uint16_t> &pdose)
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
void WaterQuality::pump_circulation(std::vector<uint16_t> &pcirc)
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
void WaterQuality::pump_mode(std::vector<uint8_t> &pmode)
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
void WaterQuality::pump_reset(std::vector<bool> &pres)
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
void WaterQuality::servo_mode(std::vector<bool> &smode)
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
void WaterQuality::servo_position(std::vector<uint8_t> &spos)
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
void WaterQuality::level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax)
{
    an.set_ResMin(rmin);
    an.set_ResMax(rmax);
}
void WaterQuality::ec(const uint8_t ch, const uint8_t type)
{
    an.set_EC_Ch(ch + 4);
    an.set_EC_Type(type);
}
void WaterQuality::ph(const uint8_t ch, const uint8_t type)
{
    an.set_PH_Ch(ch + 4);
    an.set_PH_Type(type);
}
void WaterQuality::digital_out(std::vector<bool> &dout)
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
void WaterQuality::sensor()
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
    if (this->AnInWT_Val_ != nullptr) { this->AnInWT_Val_->publish_state(an.get_WT_Val()); }
    if (this->AnInVPow_Val_ != nullptr) { this->AnInVPow_Val_->publish_state(an.get_VPow_Val()); }
    if (this->AnInLvl_Perc_ != nullptr) 
    {
        float* lvl = an.get_Lvl_Perc();
        for (size_t i = 0; i < 2; i++)
        if (i > 0)
        ap << "," << std::fixed << std::setprecision(2) << lvl[i];
        else
        ap << std::fixed << std::setprecision(2) << lvl[i];

        this->AnInLvl_Perc_->publish_state(ap.str());
    }
    if (this->AnInEC_Val_ != nullptr) { this->AnInEC_Val_->publish_state(an.get_EC_Val()); }
    if (this->AnInPH_Val_ != nullptr) { this->AnInPH_Val_->publish_state(an.get_PH_Val()); }
    if (this->AnInGen_Val_ != nullptr) 
    {
        float* gen = an.get_Gen_Val();
        for (size_t i = 0; i < 2; i++)
        if (i > 0)
        av << "," << std::fixed << std::setprecision(2) << gen[i];
        else
        av << std::fixed << std::setprecision(2) << gen[i];
    
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