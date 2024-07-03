#include "wq_analog.h"

namespace esphome {
namespace water_quality {

// WaterQuality my;

void Average(float value[]);
void WatTemp(Analog* analog, float volt);
void VoltPow(Analog* analog, float volt);
void Lvl(Analog* analog, float volt[]);
void PH(Analog* analog, float volt);
void EC(Analog* analog, float volt);
void Gen(Analog* analog, float volt[]);

void Analog::Analog_Input_Driver(float volts[])
{
    // my.ADS1115_Driver(volts);

    float ph_cal_volt = volts[get_PH_Ch() + 3]; // Read the PH voltage
    set_PH_Cal_Volt(ph_cal_volt);
    float ec_cal_volt = volts[get_EC_Ch() + 3]; // Read the EC voltage
    set_EC_Cal_Volt(ec_cal_volt);

    Average(volts);

    //Water Temperature
    WatTemp(this, volts[0]);
    
    //Voltage Power
    VoltPow(this, volts[1]);

    //Level
    Lvl(this, volts);

    //pH
    PH(this, volts[get_PH_Ch() + 3]);

    //EC
    EC(this, volts[get_EC_Ch() + 3]);

    //Analog general
    Gen(this, volts);
}

void WatTemp(Analog* analog, float volt)
{
    float model_multiply = analog->get_WatTemp_Res() / 1000.0; // Multiplier of the resistance of the temperature sensor model relative to the pt1000 sensor
    float volt_div_res = (volt * 1000.0) / (5.0 - volt) * model_multiply; // R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
    
    float wattemp_res_max = 3904.8;
    float wattemp_res_min = 185.2;
    if (volt_div_res > wattemp_res_max) // Max temp limit and set model multiplier 
        volt_div_res = wattemp_res_max * model_multiply;
    else if (volt_div_res < wattemp_res_min) // Min temp limit and set model multiplier 
        volt_div_res = wattemp_res_min * model_multiply;
    else
        volt_div_res = volt_div_res * model_multiply;
    
    // Formula                                                                                        _________________________
    float wattemp = (sqrt((-0.00232 * volt_div_res) + 17.59246) - 3.908) / (-0.00116); // Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116
    
    analog->set_WatTemp_Val(wattemp);
}
void VoltPow(Analog* analog, float volt)
{
    float voltpow = volt * 6; // Vin = Vout * (R1 + R2) / R2; (R1 = 10k & R2 = 2k)
    analog->set_VoltPow_Val(voltpow);
}
void Lvl(Analog* analog, float volt[])
{
    float lvl[2], lvlVmin[2], lvlVmax[2];
    uint16_t *resMin = analog->get_ResMin(), *resMax = analog->get_ResMax();
    uint16_t lvlVolt; // Supply voltage for the voltage divider (V)
    uint16_t lvlRes; // Resistance of the lower resistor in the voltage divider (Ω)

    // Version check
    if (analog->get_version() == 0) { lvlVolt = analog->get_VoltPow_Val(); lvlRes = 1000; } // Prototype version, Vs = Vin & R = 1k
    if (analog->get_version() == 1) { lvlVolt = 5; lvlRes = 270; } // First version, Vs = 5V & R = 270Ω

    lvlVmin[0] = lvlVolt * resMin[0] / (lvlRes + resMin[0]); // Vout = Vin * R2 / (R1 + R2)
    lvlVmin[1] = lvlVolt * resMin[1] / (lvlRes + resMin[1]);
    lvlVmax[0] = lvlVolt * resMax[0] / (lvlRes + resMax[0]);
    lvlVmax[1] = lvlVolt * resMax[1] / (lvlRes + resMax[1]);

    lvl[0] = 100 * (volt[2] - lvlVmin[0]) / (lvlVmax[0] - lvlVmin[0]);
    lvl[1] = 100 * (volt[3] - lvlVmin[1]) / (lvlVmax[1] - lvlVmin[1]);
    analog->set_Lvl_Perc(lvl);
}
void PH(Analog* analog, float volt)
{
    float temperature = analog->get_WatTemp_Val();

    // Water
    // temp    ph
    // 0       7.47
    // 10      7.27
    // 20      7.08
    // 25      7.00
    // 30      6.92
    // 40      6.77
    // 50      6.63
    // 100     6.14
    
    // // Sıcaklık telafisi için Nernst sabiti
    // float T = temperature + 273.15; // Kelvin cinsinden sıcaklık
    // const float R = 8.314; // Gaz sabiti, J/(mol*K)
    // const float F = 96485.332; // Faraday sabiti, C/mol
    // const float n = 1; // Elektron sayısı (pH ölçümünde genellikle 1)
    // //float k = (R * T) / F * 1000; // mV başına değişim (2.303 * R * T / F)

    // float acidPH = 4;
    // float neutralPH = 7;
    // float basePH = 10;

    // float acidVoltage = 0.61;
    // float neutralVoltage = 1.96;
    // float baseVoltage = 3.31;

    // float phFirst = 0, phSecond = 0;
    // float phVolt1 = 0, phVolt2 = 0;
    // if (get_PH_Calibration())
    // {
    //     if (phVolt1 == 0)
    //     {
    //         phFirst = get_PH_Cal();
    //         // Nernst denklemi
    //         phVolt1 = volt + phFirst * R * T / (n * F) * log(10); // E0 = E + pH * R * T / (n * F) * ln10
    //     }
    //     else if (phVolt2 == 0)
    //     {
    //         phSecond = get_PH_Cal();
    //         // Nernst denklemi
    //         phVolt2 = volt + phSecond * R * T / (n * F) * log(10); // E0 = E + pH * R * T / (n * F) * ln10
    //     }

    //     set_PH_Calibration(0);
    // }

    // // Nernst denklemiyle pH hesaplama
    // float pHcalc = (volt - phVolt1) / (R * T / (n * F) * log(10)); // pH = (E - E0) / (R * T / (n * F) * ln10)
    // // Sıcaklık telafisi ekleme
    // float temperatureCoefficient = (R * T) / (n * F) * log(10); // ~=0.03
    // float _phValue = phValue + (temperature - 25.0) * temperatureCoefficient;

    float (*phCal)[2] = analog->get_PH_Cal();
    float phVal1 = phCal[0][0], phVolt1 = phCal[0][1];
    float phVal2 = phCal[1][0], phVolt2 = phCal[1][1];

    // Calculate the slope between two points
    float slope = (phVal1 - phVal2) / (phVolt1 - phVolt2); // m = (y2 - y1) / (x2 - x1)
    // Calculate the y-intercept
    float intercept = phVal1 - slope * phVolt1; // b = y1 - m * x1 | b = y2 - m * x2
    // Verilen voltaj için pH değerini hesaplama
    float phValue = abs(slope * (volt) + intercept); // y = m * x + b
    
    analog->set_PH_Val(phValue);
}
void EC(Analog* analog, float volt)
{
    float RES2 = 820.0;
    float ECREF = 200.0;

    float temperature = analog->get_WatTemp_Val();

    float (*ecCal)[2] = analog->get_EC_Cal();
    float kvalueLow = ecCal[0][0], ecVolt1 = ecCal[0][1];
    float kvalueHigh = ecCal[1][0], ecVolt2 = ecCal[1][1];
    static float kvalue; 
    if (kvalue == 0) kvalue = kvalueLow; // set default K value: K = kvalueLow

    float rawEC = 10 * volt / RES2 / ECREF;
    float valueTemp = rawEC * kvalue;
    //automatic shift process
    //First Range:(0,2); Second Range:(2,20)
    if (valueTemp > 2.5)
        kvalue = kvalueHigh;
    else if (valueTemp < 2.0)
        kvalue = kvalueLow;

    float ecvalue = rawEC * kvalue; //calculate the EC value after automatic shift
    ecvalue /= (1.0 + 0.0185 * (temperature - 25.0)); //temperature compensation
    
    analog->set_EC_Val(ecvalue);
    // ESP_LOGI(TAG,"EC = %f", analog->get_EC_Val());
    // ESP_LOGI(TAG,"ec volt = %f", volt);
}
void Gen(Analog* analog, float volt[])
{
    float gen[2];
    float* ch = analog->get_Gen_Ch(); 
    uint8_t tot = analog->get_EC_Ch() + analog->get_PH_Ch();
    uint8_t rnd = round((10 - tot) / 2);

    if (!ch[0])
    {
        uint8_t ch1 = 10 - tot - rnd - 1;

        if (ch1 == analog->get_EC_Ch())
            ch[0] = ch1 - 1;
        else
            ch[0] = ch1;
    }

    if (!ch[1])
    {
        uint8_t ch2 = 10 - tot - ch[0];

        if (ch2 == analog->get_PH_Ch())
            ch[1] = ch2 + 1;
        else
            ch[1] = ch2;
    }

    gen[0] = volt[static_cast<uint8_t>(ch[0]) + 3];
    gen[1] = volt[static_cast<uint8_t>(ch[1]) + 3];
    analog->set_Gen_Ch(ch);
    analog->set_Gen_Val(gen);
}

#define NUM_SAMPLE 20

static float history[8][NUM_SAMPLE] = {}; // Static 2D array to hold the last 20 values for each element
static uint8_t counts[8] = {0}; // Static array to hold the count of values for each element
void Average(float value[])
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        // Shift values left
        for (uint8_t j = 1; j < NUM_SAMPLE; j++)
            history[i][j - 1] = history[i][j];
            
        // Add the new value to the end
        history[i][NUM_SAMPLE - 1] = value[i];
        // Increment the count of values if less than NUM_SAMPLE
        if (counts[i] < NUM_SAMPLE)
            counts[i]++;
        
        // Calculate the average
        float sum = 0;
        for (uint8_t j = 0; j < counts[i]; j++)
            sum += history[i][j];
            
        value[i] = sum / std::min(counts[i], static_cast<uint8_t>(NUM_SAMPLE));
    }
}

}  // namespace water_quality
}  // namespace esphome