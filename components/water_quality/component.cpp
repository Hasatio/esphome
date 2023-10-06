#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";

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

    String Pump_TimeConstant[6];
    uint8_t Pump_CalibX[8], Pump_CalibY[8], Pump_Mode[4], Pump_Dose[4], Pump_Total[4], Pump_Status[4]; 
    uint16_t Pump_Circulation[2];
    bool Pump_Reset[6];

    uint8_t Servo_Mode[8], Servo_Status[8];
    uint16_t Servo_Position[8];

    uint16_t AnIn_LvlResMin[2], AnIn_LvlResMax[2], AnOut_LvlPerc[2], AnIn_TempRes = 1000; //temperature sensor model pt1000 and its resistance is 1k
    float AnOut_Vcc, AnOut_Temp, TempRes;
    uint16_t AnOut_SensPerc[4];

    uint8_t DigIn_FilterCoeff[4][10];
    bool DigIn_Status[4], DigOut_Status[4]; 

void MyComponent::dat(std::vector<uint8_t> &data){}

void MyComponent::pump(String PT[6],uint8_t PCX[8],uint8_t PCY[8],uint8_t PM[4],uint8_t PD[4])
{
    for(int i = 0; i < 6; i++)
    {
        Pump_TimeConstant[i] = PT[i];
        ESP_LOGD(TAG,"%s", Pump_TimeConstant[i]);
    }
    for(int i = 0; i < 8; i++)
    {
        Pump_CalibX[i] = PCX[i];
        Pump_CalibY[i] = PCY[i];
        ESP_LOGD(TAG,"%d", Pump_CalibX[i]);
    }
    for(int i = 0; i < 4; i++)
    {
        Pump_Mode[i] = PM[i];
        Pump_Dose[i] = PD[i];
    }
}

void MyComponent::tcaselect(uint8_t bus){
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}

void MyComponent::setup() 
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    Wire.begin(SDA,SCL,freq);

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
//  ADS1115
    
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685

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

void MyComponent::loop() 
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    tcaselect(0);
    for(int i = 0; i < 4; i++)
    {
        adc[i] = ads1.readADC_SingleEnded(i%4);
        volts[i] = ads1.computeVolts(adc[i]);
        ESP_LOGD(TAG,"ads%d = %f", i+1, volts[i]);
    }
    for(int i = 4; i < 8; i++)
    {
        adc[i] = ads2.readADC_SingleEnded(i%4);
        volts[i] = ads2.computeVolts(adc[i]);
        ESP_LOGD(TAG,"ads%d = %f", i+1, volts[i]);
    }

    TempRes = (float)(volts[0] * 1000) / (5 - volts[0]) * (AnIn_TempRes / 1000); //R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
    AnOut_Temp = (float)(sqrt((-0.00232 * TempRes) + 17.59246) - 3.908) / (-0.00116)  ; //Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116
    AnOut_Vcc = (float)volts[1] * 6; //Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k

    AnOut_LvlPerc[0] = (int)volts[2] * 100 / 5 * AnIn_LvlResMax[0] / (1000 + AnIn_LvlResMax[0]) - 5 * AnIn_LvlResMin[0] / (1000 + AnIn_LvlResMin[0]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    AnOut_LvlPerc[1] = (int)volts[3] * 100 / 5 * AnIn_LvlResMax[1] / (1000 + AnIn_LvlResMax[1]) - 5 * AnIn_LvlResMin[1] / (1000 + AnIn_LvlResMin[1]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    
    ESP_LOGD(TAG,"Temp = %f", AnOut_Temp);
    ESP_LOGD(TAG,"Vcc = %f", AnOut_Vcc);
    ESP_LOGD(TAG,"Lvl1 = %d", AnOut_LvlPerc[0]);
    ESP_LOGD(TAG,"Lvl2 = %d", AnOut_LvlPerc[2]);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

    tcaselect(0);
    mcp.digitalWrite(4,LOW);
    mcp.digitalWrite(5,LOW);
    mcp.digitalWrite(6,LOW);
    mcp.digitalWrite(7,LOW);

    for(int i = 0; i < 4; i++)
    {
        DigIn_Status[i] = mcp.digitalRead(i);
        // ESP_LOGD(TAG,"dig input %d = %d", i, DigIn_Status[i]);
    }

    for(int i = 0; i < 4; i++)
    {
        if (DigOut_Status[i] == 1)
        {
            mcp.digitalWrite(i+4, HIGH);
        }
        else
        { 
            mcp.digitalWrite(i+4, LOW);
        }
        // ESP_LOGD(TAG,"dig output %d = %d", i, DigOut_Status[i]);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685

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
delay(1000);
}


}  // namespace water_quality
}  // namespace esphome