#include "water_quality.h"
#include "water_quality_i2c.h"

namespace esphome {
namespace water_quality {

    Analog ana;
    Digital digi;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

    Adafruit_MCP23X08 mcp;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void EC10() {/*DFRobot_EC10 ec;*/}
// void EC() {DFRobot_EC ec;}
    
    DFRobot_EC ec;
    DFRobot_PH ph;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

void tcaselect(uint8_t bus)
{
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ADS1115_Setup()
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
}
void ADS1115_Driver()
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

void MCP23008_Setup()
{
    if (!mcp.begin_I2C(MCP23008_ADDRESS, &Wire)) 
    {
        ESP_LOGE(TAG,"Failed to initialize MCP23008.");
        while (1);
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
void MCP23008_Driver()
{
    mcp.digitalWrite(4,LOW);
    mcp.digitalWrite(5,LOW);
    mcp.digitalWrite(6,LOW);
    mcp.digitalWrite(7,LOW);

    for(size_t i = 0; i < 4; i++)
    {
        digi.DigIn_Read[i] = mcp.digitalRead(i);
        // ESP_LOGD(TAG,"dig input %d = %d", i, DigIn_Read[i]);
    }

    for(size_t i = 0; i < 4; i++)
    {
        if (digi.DigOut_Status[i] == 1)
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

}  // namespace water_quality
}  // namespace esphome