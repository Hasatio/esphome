#include "water_quality.h"

namespace esphome {
namespace water_quality {

// MyComponent::TAG = "mycomponent";

    Analog an;
    Digital dig;
    Pump pump;
    Servo ser;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void EC10() {/*DFRobot_EC10 ec;*/}
// void EC() {DFRobot_EC ec;}
    
    DFRobot_EC ec;
    DFRobot_PH ph;
    
static unsigned long timepoint = millis();

void MyComponent::Analog_Input_Driver()
{
    // ads1115();
    uint8_t tot, rnd, AnInGen_Ch[2];
    tot = an.AnInEC_Ch + an.AnInPH_Ch;
    rnd = round((10 - tot) / 2);
    AnInGen_Ch[0] = 10 - tot - rnd - 1;
    AnInGen_Ch[1] = 10 - tot - AnInGen_Ch[0];
    AnInGen_Ch[0] = (AnInGen_Ch[0] == an.AnInEC_Ch)? AnInGen_Ch[0] - 1 : AnInGen_Ch[0];
    AnInGen_Ch[1] = (AnInGen_Ch[1] == an.AnInPH_Ch)? AnInGen_Ch[1] + 1 : AnInGen_Ch[1];

        // ESP_LOGD(TAG,"ads = %f", volts[3+4]);
        // ESP_LOGD(TAG,"ads1 = %f", (ads2.readADC_SingleEnded(3)/10));
        // delay(1000);
    an.VPow = (float)an.volts[1] * 6; //Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k
    an.LvlPerc[0] = (float)an.volts[2] * 100 / 5 * an.AnInLvl_ResMax[0] / (1000 + an.AnInLvl_ResMax[0]) - 5 * an.AnInLvl_ResMin[0] / (1000 + an.AnInLvl_ResMin[0]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    an.LvlPerc[1] = (float)an.volts[3] * 100 / 5 * an.AnInLvl_ResMax[1] / (1000 + an.AnInLvl_ResMax[1]) - 5 * an.AnInLvl_ResMin[1] / (1000 + an.AnInLvl_ResMin[1]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k

    // EC = volts[AnInEC_Ch];
    // PH = volts[AnInPH_Ch];
    an.AnGen[0] = an.volts[AnInGen_Ch[0] + 3];
    an.AnGen[1] = an.volts[AnInGen_Ch[1] + 3];
}
void MyComponent::setup()
{
    // Wire.begin();

    an.ads1115_set();
    dig.mcp23008_set();
    pca9685_set();
}
void MyComponent::dump_config()
{
    LOG_I2C_DEVICE(this);
    if (this->is_failed())
    ESP_LOGE(TAG, "Communication failed!");
    
        ESP_LOGD(TAG,"ads0 = %f", an.volts[0]);
        ESP_LOGD(TAG,"ads1 = %f", an.volts[1]);
        ESP_LOGD(TAG,"ads2 = %f", an.volts[2]);
        ESP_LOGD(TAG,"ads3 = %f", an.volts[3]);
        ESP_LOGD(TAG,"ads4 = %f", an.volts[4]);
        ESP_LOGD(TAG,"ads5 = %f", an.volts[5]);
        ESP_LOGD(TAG,"ads6 = %f", an.volts[6]);
        ESP_LOGD(TAG,"ads7 = %f", an.volts[7]);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    // Wire.begin(SDA,SCL,frq);

    // for (uint8_t t=0; t<8; t++) 
    // {
    //   mux.tcaselect(t);
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
    dig.mcp23008();
    pca9685();
    // pump_total();
    an.ads1115();
    sensor();
    Analog_Input_Driver();
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
    if (this->AnInWT_Val_ != nullptr) { this->AnInWT_Val_->publish_state(an.WT); }
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