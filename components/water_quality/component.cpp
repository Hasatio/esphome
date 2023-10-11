#include "component.h"

namespace esphome {
namespace water_quality {

static const char *TAG = "mysensor";

    String Pump_TimeConstant[6];
    uint8_t Pump_Mode[4], Pump_Dose[4], Pump_Total[4], Pump_Status[4]; 
    uint16_t Pump_Circulation[2];
    bool Pump_Reset[6];

    uint8_t Servo_Mode[8], Servo_Status[8];
    uint16_t Servo_Position[8];

    uint16_t AnIn_LvlResMin[2], AnIn_LvlResMax[2], AnOut_LvlPerc[2], AnIn_TempRes = 1000; //temperature sensor model pt1000 and its resistance is 1k
    float AnOut_Vcc, AnOut_Temp, TempRes;
    uint16_t AnOut_SensPerc[4];

    uint8_t DigIn_FilterCoeff[4][10];
    bool DigIn_Status[4], DigOut_Status[4]; 


void MyComponent::setup() 
{

}

void MyComponent::loop() 
{

    // Pump_CalibX = int(custom_data);
    for (size_t i = 0; i < 8; i++)
    {
        ESP_LOGD(TAG,"x1[%d] = %d", i, Pump_Calib_X1[i]);
        ESP_LOGD(TAG,"y1[%d] = %d", i, Pump_Calib_Y1[i]);
        ESP_LOGD(TAG,"x2[%d] = %d", i, Pump_Calib_X2[i]);
        ESP_LOGD(TAG,"y2[%d] = %d", i, Pump_Calib_Y2[i]);
        ESP_LOGD(TAG,"x3[%d] = %d", i, Pump_Calib_X3[i]);
        ESP_LOGD(TAG,"y3[%d] = %d", i, Pump_Calib_Y3[i]);
        ESP_LOGD(TAG,"x4[%d] = %d", i, Pump_Calib_X4[i]);
        ESP_LOGD(TAG,"y4[%d] = %d", i, Pump_Calib_Y4[i]);
    }
    
        

delay(1000);
}


}  // namespace water_quality
}  // namespace esphome