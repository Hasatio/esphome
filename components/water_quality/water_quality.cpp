#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_analog.h"
#include "wq_digital.h"
#include "wq_pump.h"
#include "wq_servo.h"


// // To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
// #include "ESP32TimerInterrupt.h"

namespace esphome {
namespace water_quality {

    Analog an;
    Digital dig;
    Pump pump;
    Servo ser;
        
// static unsigned long timepoint = millis();

// // Init ESP32 timer 0
// ESP32Timer ITimer0(0);

volatile int interruptCounter;  //for counting interrupt
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void WaterQuality::setup()
{
    ADS1115_Setup(ADS1X15_ADDRESS1);
    ADS1115_Setup(ADS1X15_ADDRESS2);
    MCP23008_Setup(MCP23008_ADDRESS);
    PCA9685_Setup(PCA9685_I2C_ADDRESS);	
    
    // // Interval in microsecs
	// if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
	// {
	// 	ESP_LOGCONFIG(TAG, "Starting  ITimer0 OK, millis() = %d", millis());
	// }
	// else
	// 	ESP_LOGCONFIG(TAG, "Can't set ITimer0. Select another freq. or timer");

    timer = timerBegin(0, 80, true);           	// timer 0, prescalar: 80, UP counting
    timerAttachInterrupt(timer, &onTimer, true); 	// Attach interrupt
    timerAlarmWrite(timer, 1000000, true);  		// Match value= 1000000 for 1 sec. delay.
    timerAlarmEnable(timer);           			// Enable Timer with interrupt (Alarm Enable)


}
void WaterQuality::dump_config()
{
    LOG_I2C_DEVICE(this);
    if (this->is_failed())
        ESP_LOGE(TAG, "Communication failed!");
    else
        ESP_LOGI(TAG, "Communication Successfulled!");
        
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
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    // Wire.begin(SDA,SCL,frq);

    // for (size_t t=0; t<8; t++) 
    // {
    //   tcaselect(t);
    //   ESP_LOGI(TAG, "TCA Port %d", t);

    //   for (uint8_t addr = 0; addr<=127; addr++) 
    //   {
    //     if (addr == TCA9548_ADDRESS) continue;

    //     Wire.beginTransmission(addr);
    //     if (!Wire.endTransmission()) 
    //     {
    //       ESP_LOGI(TAG, "Found I2C 0x%x",addr);
    //     }
    //   }
    // }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    uint8_t dose = 0, circ = 0;
    float* calib = pump.get_Pump_Calib_Gain();
    uint8_t* type = pump.get_Pump_Type();

    for (size_t i = 0; i < 4; i++)
        if (type[i] == 1)
            dose += 1;
        else if (type[i] == 2)
            circ += 1;

    ESP_LOGI(TAG, "Pump_dose = %d", dose);
    ESP_LOGI(TAG, "Pump_circ = %d", circ);
    for (size_t i = 0; i < 6; i++)
        ESP_LOGI(TAG, "Pump_Calib_Gain[%d] = %.2f", i, calib[i]);
    for (size_t i = 0; i < 6; i++)
        ESP_LOGI(TAG, "Pump_Type[%d] = %d", i, type[i]);

    uint16_t *resmin = an.get_ResMin(), *resmax = an.get_ResMax();
    for (size_t i = 0; i < sizeof(resmin) / sizeof(resmin[0]); i++)
        ESP_LOGI(TAG, "ResMin[%d] = %d, ResMax[%d] = %d", i, resmin[i], i, resmax[i]);
    ESP_LOGI(TAG, "EC_ch = %d, EC_type = %d", an.get_EC_Ch(), an.get_EC_Type());
    ESP_LOGI(TAG, "PH_ch = %d, PH_type = %d", an.get_PH_Ch(), an.get_PH_Type());
}
void WaterQuality::loop() 
{
    if (interruptCounter > 0)
    {
 
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);
        
        totalInterruptCounter++;         	//counting total interrupt
    }

    // static uint32_t lastTime = 0;
	// static uint32_t lastChangeTime = 0;
	// static uint32_t currTime;
	// static uint32_t multFactor = 0;

	// currTime = millis();

	// if (millis() - lastTime >= CHECK_INTERVAL_MS)
	// {

	// 		//setInterval(unsigned long interval, timerCallback callback)
	// 		multFactor = 2;

	// 		ITimer0.setInterval(TIMER0_INTERVAL_MS * 1000 * multFactor, TimerHandler0);
    
	// 		ESP_LOGI(TAG, "Changing Interval, Timer0 = %d", millis() - lastTime);

	// 	lastTime = millis();
	// }


	// static unsigned long lastTimer0 = 0;

	// static bool timer0Stopped         = false;

	// if (millis() - lastTimer0 > TIMER0_DURATION_MS)
	// {
	// 	lastTimer0 = millis();

	// 	if (timer0Stopped)
	// 	{
    //         ESP_LOGI(TAG, "Start ITimer0, millis() = %d", millis());
	// 		ITimer0.restartTimer();
	// 	}
	// 	else
	// 	{
    //         ESP_LOGI(TAG, "Stop ITimer0, millis() = %d", millis());
	// 		ITimer0.stopTimer();
	// 	}

	// 	timer0Stopped = !timer0Stopped;
	// }



    // delay(500);
    // ESP_LOGI(TAG, "WT = %d", an.get_WT_Val());
}

    static uint32_t lastTime = 0;
	static uint32_t lastChangeTime = 0;
	static uint32_t currTime;
	static uint32_t multFactor = 0;

bool IRAM_ATTR WaterQuality::TimerHandler0(void * timerNo)
{

ESP_LOGI(TAG, "TimerHandler0");

	return true;
}
void IRAM_ATTR WaterQuality::onTimer()
{
portENTER_CRITICAL_ISR(&timerMux);
interruptCounter++;
portEXIT_CRITICAL_ISR(&timerMux);

ESP_LOGI(TAG, "TimerHandler0");
}

float a[8], p[16];
bool d[4];
void WaterQuality::update()
{
	currTime = millis();

    ADS1115_Driver(a);
    an.Analog_Input_Driver(a);

    dig.Digital_Output_Driver(d);
    MCP23008_Driver(d);
    dig.Digital_Input_Driver(d);

lastChangeTime = millis();
    
    pump.Pump_driver(p);
    ser.Servo_driver(p);
    PCA9685_Driver(p);

lastTime = millis();

    // ESP_LOGI(TAG, "diğer fonk süresi = %d", lastChangeTime - currTime);
    // ESP_LOGI(TAG, "pump süresi = %d", lastTime - lastChangeTime);
    // ESP_LOGI(TAG, "toplam süre = %d", lastTime - currTime);

    sensor();
    
    // an.set_WT_Val(1.23);
    // ESP_LOGD(TAG, "test = %f", request_measurement());
    // ESP_LOGD(TAG, "vpow test = %f", an.get_VPow_Val());
    
}

void WaterQuality::version(const uint8_t ver)
{
    an.set_version(ver);
}
void WaterQuality::pump_calib_gain(const std::vector<float> &pcal)
{
    float pcal_[6];

    for (size_t i = 0; i < 6; i++)
    {
        pcal_[i] = pcal[i] / 60;
    }

    pump.set_Pump_Calib_Gain(pcal_);
}
void WaterQuality::pump_type(const std::vector<uint8_t> &ptype)
{
    uint8_t ptype_[6];
    
    for (size_t i = 0; i < 6; i++)
    {
        ptype_[i] = ptype[i];
    }

    pump.set_Pump_Type(ptype_);
}
void WaterQuality::pump_mode(std::vector<uint8_t> &pmode)
{
    uint8_t* pmode_ = pump.get_Pump_Mode();
    std::vector<uint8_t> pm(pmode_, pmode_ + 6);

    if (pm != pmode)
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
    float* pdose_ = pump.get_Pump_Dose();
    std::vector<float> pd(pdose_, pdose_ + 6);

    if (pd != pdose)
    {
        for (size_t i = 0; i < 6; i++)
        {
            if (ptype[i] == 1)
            {
                if (pmode[i] == 1)
                    pdose_[i] += pdose[i];
                else
                    pdose_[i] = pdose[i];
                ESP_LOGD(TAG, "Pump_Dose[%d] = %d", i, pdose_[i]);
            }
        }
    }
}
void WaterQuality::pump_circulation(std::vector<float> &pcirc)
{
    uint8_t* ptype = pump.get_Pump_Type();
    uint8_t* pmode = pump.get_Pump_Mode();
    float* pcirc_ = pump.get_Pump_Circulation();
    std::vector<float> pc(pcirc_, pcirc_ + 6);

    if (pc != pcirc)
    {
        for (size_t i = 0; i < 6; i++)
        {
            if (ptype[i] == 2)
            {
                if (pmode[i] == 1)
                    pcirc_[i] += pcirc[i];
                else
                    pcirc_[i] = pcirc[i];
                ESP_LOGD(TAG, "Pump_Circulation[%d] = %d", i, pcirc_[i]);
            }
        }
    }
}
void WaterQuality::pump_reset(std::vector<bool> &pres)
{
    bool* pres_ = pump.get_Pump_Reset();
    std::vector<bool> pr(pres_, pres_ + 6);

    if (pr != pres)
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
        uint16_t (*ptot)[2] = pump.get_Pump_Total();
        std::stringstream pt;

        for (size_t i = 0; i < 6; i++)
            if (i == 0)
                pt << std::fixed << std::setprecision(4) << ptot[i][0] + ptot[i][1] / 10000.0;
            else
                pt << "," << std::fixed << std::setprecision(4) << ptot[i][0] + ptot[i][1] / 10000.0;
            
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
    if (this->AnInWT_Val_ != nullptr)   {this->AnInWT_Val_->publish_state(an.get_WT_Val());}
    if (this->AnInVP_Val_ != nullptr)   {this->AnInVP_Val_->publish_state(an.get_VP_Val());}
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
    if (this->AnInEC_Val_ != nullptr)   {this->AnInEC_Val_->publish_state(an.get_EC_Val());}
    if (this->AnInPH_Val_ != nullptr)   {this->AnInPH_Val_->publish_state(an.get_PH_Val());}
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