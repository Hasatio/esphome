#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_analog.h"
#include "wq_digital.h"
#include "wq_pump.h"
#include "wq_servo.h"

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
  MCP23008_Setup(MCP23008_ADDRESS);
  PCA9685_Setup(PCA9685_I2C_ADDRESS);
}
void WaterQuality::dump_config()
{
  LOG_I2C_DEVICE(this);
  if (this->is_failed())
    ESP_LOGE(TAG, "Communication failed!");
  else
    ESP_LOGI(TAG, "Communication Successfulled!");
      
  ESP_LOGI(TAG, "Gain: %d", this->get_gain());
  ESP_LOGI(TAG, "Continuous mode: %s", this->get_continuous_mode() ? "true" : "false");
  ESP_LOGI(TAG, "Data rate: %d", this->get_data_rate());
  ESP_LOGI(TAG, "Resolution: %d", this->get_resolution());

  ESP_LOGI(TAG, "Digital status: %x", this->olat_);

  this->set_i2c_address(MCP23008_ADDRESS);
  uint8_t iodir, ipol, gpinten, defval, intcon, iocon, gppu, intf, intcap, gpio, olat;
      ESP_LOGI(TAG, "iodir: %x", iodir);
      this->read_byte(MCP23008_IODIR, &iodir);
      ESP_LOGI(TAG, "iodir: %x", iodir);
      
      ESP_LOGI(TAG, "ipol: %x", ipol);
      this->read_byte(MCP23008_IPOL, &ipol);
      ESP_LOGI(TAG, "ipol: %x", ipol);

      ESP_LOGI(TAG, "gpinten: %x", gpinten);
      this->read_byte(MCP23008_GPINTEN, &gpinten);
      ESP_LOGI(TAG, "gpinten: %x", gpinten);
      
      ESP_LOGI(TAG, "defval: %x", defval);
      this->read_byte(MCP23008_DEFVAL, &defval);
      ESP_LOGI(TAG, "defval: %x", defval);
      
      ESP_LOGI(TAG, "intcon: %x", intcon);
      this->read_byte(MCP23008_INTCON, &intcon);
      ESP_LOGI(TAG, "intcon: %x", intcon);
      
      ESP_LOGI(TAG, "iocon: %x", iocon);
      this->read_byte(MCP23008_IOCON, &iocon);
      ESP_LOGI(TAG, "iocon: %x", iocon);
      for (size_t i = 0; i < 8; i++)
      {  
        this->read_byte(MCP23008_IODIR, &iodir);
        ESP_LOGI(TAG, "Digital pin(%d): %d", i, iodir & (1 << i));
      }
      this->read_byte(MCP23008_IOCON, &iocon);
      ESP_LOGI(TAG, "iocon: %x", iocon);

      ESP_LOGI(TAG, "gppu: %x", gppu);
      this->read_byte(MCP23008_GPPU, &gppu);
      ESP_LOGI(TAG, "gppu: %x", gppu);
      
      ESP_LOGI(TAG, "intf: %x", intf);
      this->read_byte(MCP23008_INTF, &intf);
      ESP_LOGI(TAG, "intf: %x", intf);
      
      ESP_LOGI(TAG, "intcap: %x", intcap);
      this->read_byte(MCP23008_INTCAP, &intcap);
      ESP_LOGI(TAG, "intcap: %x", intcap);
      
      ESP_LOGI(TAG, "gpio: %x", gpio);
      this->read_byte(MCP23008_GPIO, &gpio);
      ESP_LOGI(TAG, "gpio: %x", gpio);
      
      ESP_LOGI(TAG, "olat: %x", olat);
      this->read_byte(MCP23008_OLAT, &olat);
      ESP_LOGI(TAG, "olat: %x", olat);
    
<<<<<<< HEAD
=======

  ESP_LOGCONFIG(TAG, "PCA9685:");
  ESP_LOGCONFIG(TAG, "  Mode: 0x%02X", this->mode_);
  if (this->extclk_) {
    ESP_LOGCONFIG(TAG, "  EXTCLK: enabled");
  } else {
    ESP_LOGCONFIG(TAG, "  EXTCLK: disabled");
    ESP_LOGCONFIG(TAG, "  Frequency: %.0f Hz", this->frequency_);
  }
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Setting up PCA9685 failed!");
  }
  
ESP_LOGD(TAG, "min_channel: %d", this->min_channel_);
ESP_LOGD(TAG, "max_channel: %d", this->max_channel_);
ESP_LOGD(TAG, "update: %d", this->update_);
for (size_t i = 0; i < 8; i++)
{   
  ESP_LOGD(TAG, "pwm_amounts%d: %d", i, this->pwm_amounts_[i]);
}
>>>>>>> parent of 3b0422f (pump and whole installation)
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

<<<<<<< HEAD
    uint8_t dose = 0, circ = 0;
    uint8_t* calib = pump.get_Pump_Calib_Gain();
    uint8_t* type = pump.get_Pump_Type();
    for (size_t i = 0; i < 4; i++)
        if (type[i] == 1)
            dose += 1;
        else if (type[i] == 2)
            circ += 1;
=======
  ESP_LOGI(TAG,"Pump_dose = %d", pump.dose);
  ESP_LOGI(TAG,"Pump_circ = %d", pump.circ);
>>>>>>> parent of 3b0422f (pump and whole installation)

  for (size_t i = 0; i < pump.Pump_Type.size(); i++)
  {
    ESP_LOGI(TAG,"Pump_Calib_Gain[%d] = %.2f", i, pump.Pump_Calib_Gain[i]);
  }

<<<<<<< HEAD
    for (size_t i = 0; i < 6; i++)
    {
        ESP_LOGI(TAG,"Pump_Calib_Gain[%d] = %.2f", i, calib[i]);
    }

    for (size_t i = 0; i < 6; i++)
    {
        ESP_LOGI(TAG,"Pump_Type[%d] = %d", i, type[i]);
    }
=======
  for (size_t i = 0; i < pump.Pump_Type.size(); i++)
  {
    ESP_LOGI(TAG,"Pump_Type[%d] = %d", i, pump.Pump_Type[i]);
    ESP_LOGI(TAG,"Pump_Total[%d] = %d.%d", i, pump.Pump_Total[i][0], pump.Pump_Total[i][1]);
  }

  uint16_t *resmin = an.get_ResMin(), *resmax = an.get_ResMax();
  for (size_t i = 0; i < sizeof(resmin) / sizeof(resmin[0]); i++)
  {
    ESP_LOGI(TAG,"ResMin[%d] = %d", i, resmin[i]);
    ESP_LOGI(TAG,"ResMax[%d] = %d", i, resmax[i]);
  }
>>>>>>> parent of 3b0422f (pump and whole installation)

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
  float a[8];
  bool d[4];

  ADS1115_Driver(a);
  an.Analog_Input_Driver(a);

  dig.Digital_Output_Driver(d);
  MCP23008_Driver(d);
  dig.Digital_Input_Driver(d);

<<<<<<< HEAD
    // pump.Pump_driver(p);
    ser.Servo_driver(p);
    PCA9685_Driver(p);

    sensor();
    
    // an.set_WT_Val(1.23);
    // ESP_LOGD(TAG,"test = %f", request_measurement());
    // ESP_LOGD(TAG,"vpow test = %f", an.get_VPow_Val());
    
}

void WaterQuality::version(const uint8_t ver)
=======
  PCA9685_Driver(0, (float)pump.Pump_Dose[0]/100);
  PCA9685_Driver(1, (float)pump.Pump_Dose[1]/100);
  PCA9685_Driver(8, (float)pump.Pump_Dose[6]/100);
  
  // pump_total();
  sensor();
  // an.set_WT_Val(1.23);
  // ESP_LOGD(TAG,"test = %f", request_measurement());
  // ESP_LOGD(TAG,"vpow test = %f", an.get_VPow_Val());
    
}

  bool pd[6], pc[6];

void WaterQuality::version(const uint8_t v)
>>>>>>> parent of 3b0422f (pump and whole installation)
{
  an.ver = v;
}
void WaterQuality::pump_calib_gain(const std::vector<float> &pcal)
{
<<<<<<< HEAD
    uint8_t pcal_[6];

    for (size_t i = 0; i < 6; i++)
    {
        pcal_[i] = pcal[i];
    }

    pump.set_Pump_Calib_Gain(pcal_);
=======
  pump.Pump_Calib_Gain = pcg;
>>>>>>> parent of 3b0422f (pump and whole installation)
}
void WaterQuality::pump_type(const std::vector<uint8_t> &ptype, const uint8_t d, const uint8_t c)
{
<<<<<<< HEAD
    uint8_t ptype_[6];
    
    for (size_t i = 0; i < 6; i++)
    {
        ptype_[i] = ptype[i];
    }

    pump.set_Pump_Type(ptype_);
}
void WaterQuality::pump_mode(std::vector<uint8_t> &pmode)
{
    std::vector<uint8_t> pm(pump.get_Pump_Mode(), pump.get_Pump_Mode() + 6);
    uint8_t pmode_[6];

    if (pm != pmode)
    {
        for (size_t i = 0; i < 6; i++)
        {
            pmode_[i] = pmode[i];
            ESP_LOGD(TAG,"Pump_Mode[%d] = %d", i, pmode_[i]);
        }

        pump.set_Pump_Mode(pmode_);
    }
}
void WaterQuality::pump_dose(std::vector<uint16_t> &pdose)
{
    std::vector<uint16_t> pd(pump.get_Pump_Dose(), pump.get_Pump_Dose() + 6);
    uint16_t* pdose_ = pump.get_Pump_Dose();

    if (pd != pdose)
    {
        for (size_t i = 0; i < 6; i++)
        {
            pdose_[i] += pdose[i];
            ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pdose_[i]);
        }

        pump.set_Pump_Dose(pdose_);
    }
}
void WaterQuality::pump_circulation(std::vector<uint16_t> &pcirc)
{
    std::vector<uint16_t> pc(pump.get_Pump_Circulation(), pump.get_Pump_Circulation() + 6);
    uint16_t* pcirc_ = pump.get_Pump_Circulation();

    if (pc != pcirc)
    {
        for (size_t i = 0; i < 6; i++)
        {
            pcirc_[i] += pcirc[i];
            ESP_LOGD(TAG,"Pump_Circulation[%d] = %d", i, pcirc_[i]);
        }

        pump.set_Pump_Circulation(pcirc_);
    }
}
void WaterQuality::pump_reset(std::vector<bool> &pres)
{
    std::vector<bool> pr(pump.get_Pump_Reset(), pump.get_Pump_Reset() + 6);
    bool pres_[6];

    if (pr != pres)
    {
        for (size_t i = 0; i < 6; i++)
        {
            pres_[i] = pres[i];
            ESP_LOGD(TAG,"Pump_Reset[%d] = %d", i, pres_[i]);
        }

        pump.set_Pump_Reset(pres_);
    }
}
void WaterQuality::servo_mode(std::vector<bool> &smode)
{
    std::vector<bool> sm(ser.get_Servo_Mode(), ser.get_Servo_Mode() + 8);
    bool smode_[8];
    
    if (sm != smode)
    {
        for (size_t i = 0; i < 8; i++)
        {
            smode_[i] = smode[i];
            ESP_LOGD(TAG,"Servo_Mode[%d] = %d", i, smode_[i]);
        }
    
        ser.set_Servo_Mode(smode_);
=======
  pump.dose = d;
  pump.circ = c;
  
  pump.Pump_Type = ptype;
}
void WaterQuality::pump_dose(std::vector<uint16_t> &pdose)
{
  uint8_t* ps = pump.get_Pump_Status();
  uint16_t* pd = pump.get_Pump_Dose();
  for (size_t i = 0; i < pdose.size(); i++)
  {
    if (ps[i] != 1)
    {
      pd[i] = pdose[i];
      ESP_LOGD(TAG,"Pump_Dose[%d] = %d", i, pd[i]);
    }
  }
    pump.set_Pump_Dose(pd);
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
>>>>>>> parent of 3b0422f (pump and whole installation)
    }
  }
}
void WaterQuality::servo_position(std::vector<uint8_t> &spos)
{
<<<<<<< HEAD
    std::vector<uint8_t> sp(ser.get_Servo_Position(), ser.get_Servo_Position() + 8);
    uint8_t spos_[8];
    
    if (sp != spos)
    {
        for (size_t i = 0; i < 8; i++)
        {
            spos_[i] = spos[i];
            ESP_LOGD(TAG,"Servo_Position[%d] = %d", i, spos_[i]);
        }

        ser.set_Servo_Position(spos_);
=======
  if (ser.Servo_Position != spos)
  {
    ser.Servo_Position = spos;
    for (size_t i = 0; i < ser.Servo_Position.size(); i++)
    {
      ESP_LOGD(TAG,"Servo_Position[%d] = %d", i, ser.Servo_Position[i]);
>>>>>>> parent of 3b0422f (pump and whole installation)
    }
  }
}
void WaterQuality::level_res(const std::vector<uint16_t> &rmin, const std::vector<uint16_t> &rmax)
{    
<<<<<<< HEAD
    uint16_t rminArray[2] = {0}, rmaxArray[2] = {0};

    for (size_t i = 0; i < rmin.size(); i++)
    {
        rminArray[i] = rmin[i];
        rmaxArray[i] = rmax[i];
    }

    an.set_ResMin(rminArray);
    an.set_ResMax(rmaxArray);
=======
  uint16_t rminArray[2] = {0,0}, rmaxArray[2] = {0,0};
  for (size_t i = 0; i < rmin.size(); i++)
  {
    rminArray[i] = rmin[i];
    rmaxArray[i] = rmax[i];
  }
  an.set_ResMin(rminArray);
  an.set_ResMax(rmaxArray);
>>>>>>> parent of 3b0422f (pump and whole installation)
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
<<<<<<< HEAD
    std::vector<bool> d(dig.get_Digital_Out(), dig.get_Digital_Out() + 4);
    bool dout_[4];

    if (d != dout)
    {
        for (size_t i = 0; i < 4; i++)
        {
            dout_[i] = dout[i];
            ESP_LOGD(TAG,"DigOut_Status[%d] = %d", i, dout_[i]);
        }
        dig.set_Digital_Out(dout_);
    }
=======
  std::vector<bool> dout_(dig.get_Digital_Out(), dig.get_Digital_Out() + 4);
  bool d[4];
  if (dout_ != dout)
  {
    for (size_t i = 0; i < dout.size(); i++)
    {
      d[i] = dout[i];
      ESP_LOGD(TAG,"DigOut_Status[%d] = %d", i, (int)dout[i]);
    }
    dig.set_Digital_Out(d);
  }
>>>>>>> parent of 3b0422f (pump and whole installation)
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor
void WaterQuality::sensor()
{
        uint16_t (*ptot)[6][2] = pump.get_Pump_Total();
        uint8_t* pstat = pump.get_Pump_Status();
        bool* sstat = ser.get_Servo_Status();
        float* lvl = an.get_Lvl_Perc();
        float* gen = an.get_Gen_Val();
        bool* din = dig.get_Digital_In();

        std::stringstream pt;
        std::stringstream ps;
        std::stringstream ss;
        std::stringstream ap;
        std::stringstream av;
        std::stringstream ds;

    if (this->Pump_Tot_ != nullptr)
    {
        for (size_t i = 0; i < 6; i++)
<<<<<<< HEAD
            if (i > 0)
                pt << "," << std::fixed << std::setprecision(3) << static_cast<float>((*ptot)[i][0]) + static_cast<float>((*ptot)[i][1])/1000;
            else
                pt << std::fixed << std::setprecision(3) << static_cast<float>((*ptot)[i][0]) + static_cast<float>((*ptot)[i][1])/1000;
=======
        if (i > 0)
        pt << "," << std::fixed << std::setprecision(3) << (float)pump.Pump_Total[i][0] + (float)pump.Pump_Total[i][1]/1000;
        else
        pt << std::fixed << std::setprecision(3) << (float)pump.Pump_Total[i][0] + (float)pump.Pump_Total[i][1]/1000;
>>>>>>> parent of 3b0422f (pump and whole installation)
    
        this->Pump_Tot_->publish_state(pt.str());
    }
    if (this->Pump_Stat_ != nullptr)
    { 
        for (size_t i = 0; i < 6; i++)
<<<<<<< HEAD
            if (i > 0)
                ps << "," << std::fixed << std::setprecision(0) << static_cast<uint8_t>(pstat[i]);
            else
                ps << std::fixed << std::setprecision(0) << static_cast<uint8_t>(pstat[i]);
=======
        if (i > 0)
        ps << "," << std::fixed << std::setprecision(0) << (int)pump.Pump_Status[i];
        else
        ps << std::fixed << std::setprecision(0) << (int)pump.Pump_Status[i];
>>>>>>> parent of 3b0422f (pump and whole installation)

        this->Pump_Stat_->publish_state(ps.str());
    }
    if (this->Servo_Stat_ != nullptr)
    { 
        for (size_t i = 0; i < 8; i++)
<<<<<<< HEAD
            if (i > 0)
                ss << "," << std::fixed << std::setprecision(0) << static_cast<uint8_t>(sstat[i]);
            else
                ss << std::fixed << std::setprecision(0) << static_cast<uint8_t>(sstat[i]);
=======
        if (i > 0)
        ss << "," << std::fixed << std::setprecision(0) << (int)ser.Servo_Status[i];
        else
        ss << std::fixed << std::setprecision(0) << (int)ser.Servo_Status[i];
>>>>>>> parent of 3b0422f (pump and whole installation)

        this->Servo_Stat_->publish_state(ss.str());
    }
    if (this->AnInWT_Val_ != nullptr)       {this->AnInWT_Val_->publish_state(an.get_WT_Val());}
    if (this->AnInVPow_Val_ != nullptr)     {this->AnInVPow_Val_->publish_state(an.get_VPow_Val());}
    if (this->AnInLvl_Perc_ != nullptr) 
    {
<<<<<<< HEAD
=======
        float* lvl = an.get_Lvl_Perc();
>>>>>>> parent of 3b0422f (pump and whole installation)
        for (size_t i = 0; i < 2; i++)
        if (i > 0)
        ap << "," << std::fixed << std::setprecision(2) << lvl[i];
        else
        ap << std::fixed << std::setprecision(2) << lvl[i];

        this->AnInLvl_Perc_->publish_state(ap.str());
    }
    if (this->AnInEC_Val_ != nullptr)       {this->AnInEC_Val_->publish_state(an.get_EC_Val());}
    if (this->AnInPH_Val_ != nullptr)       {this->AnInPH_Val_->publish_state(an.get_PH_Val());}
    if (this->AnInGen_Val_ != nullptr) 
    {
<<<<<<< HEAD
=======
        float* gen = an.get_Gen_Val();
>>>>>>> parent of 3b0422f (pump and whole installation)
        for (size_t i = 0; i < 2; i++)
        if (i > 0)
        av << "," << std::fixed << std::setprecision(2) << gen[i];
        else
        av << std::fixed << std::setprecision(2) << gen[i];
    
        this->AnInGen_Val_->publish_state(av.str());
    }
    if (this->DigIn_Stat_ != nullptr) 
    {
<<<<<<< HEAD
        for (size_t i = 0; i < 4; i++)
            if (i > 0)
                ds << "," << std::fixed << std::setprecision(2) << static_cast<uint8_t>(din[i]);
            else
                ds << std::fixed << std::setprecision(2) << static_cast<uint8_t>(din[i]);
=======
        bool* in = dig.get_Digital_In();
        for (size_t i = 0; i < 4; i++)
        if (i > 0)
        ds << "," << std::fixed << std::setprecision(2) << (int)in[i];
        else
        ds << std::fixed << std::setprecision(2) << (int)in[i];
>>>>>>> parent of 3b0422f (pump and whole installation)

        this->DigIn_Stat_->publish_state(ds.str());
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace water_quality
}  // namespace esphome