#include "wq_analog.h"

namespace esphome {
namespace water_quality {

// WaterQuality my;

    DFRobot_PH ph;
    DFRobot_EC ec;

void ph1(Analog* analog);
void ph2(Analog* analog);
double averageArray(float* arr, int number);

void Analog::Analog_Input_Driver(float volts[])
{
    // my.ADS1115_Driver(volts);

    //Water Temperature
    uint16_t timeperiod = 1000; // Wait time before each update

    if (millis() - get_Analog_Timepoint() >= timeperiod)
    {
        float model_multiply = get_WatTemp_Res() / 1000.0; // Multiplier of the resistance of the temperature sensor model relative to the pt1000 sensor
        float WatTemp_Res = (volts[0] * 1000.0) / (5.0 - volts[0]) * model_multiply; // R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
        
        float WatTemp_Res_Max = 3904.8;
        float WatTemp_Res_Min = 185.2;
        if (WatTemp_Res > WatTemp_Res_Max) // Max temp limit and set model multiplier 
            WatTemp_Res = WatTemp_Res_Max * model_multiply;
        else if (WatTemp_Res < WatTemp_Res_Min) // Min temp limit and set model multiplier 
            WatTemp_Res = WatTemp_Res_Min * model_multiply;
        else
            WatTemp_Res = WatTemp_Res * model_multiply;
        
        // Formula                                                                                        _________________________
        float WatTemp = (sqrt((-0.00232 * WatTemp_Res) + 17.59246) - 3.908) / (-0.00116); // Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116
        
        set_WatTemp_Val(WatTemp);
        set_Analog_Timepoint(millis());
    }
    

    //Power
    set_VoltPow_Val(volts[1] * 6); // Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k
    

    //Level
    float lvl[2], Vmin[2], Vmax[2];
    uint16_t res, volt, *resmin = get_ResMin(), *resmax = get_ResMax();

    if (get_version() == 0) { res = 1000; volt = get_VoltPow_Val(); } // Version check
    if (get_version() == 1) { res = 270; volt = 5; }

    Vmin[0] = volt * resmin[0] / (res + resmin[0]); // Vout = Vin * R2 / (R1 + R2); R1 = 10k
    Vmin[1] = volt * resmin[1] / (res + resmin[1]);
    Vmax[0] = volt * resmax[0] / (res + resmax[0]);
    Vmax[1] = volt * resmax[1] / (res + resmax[1]);

    lvl[0] = 100 * (volts[2] - Vmin[0]) / (Vmax[0] - Vmin[0]);
    lvl[1] = 100 * (volts[3] - Vmin[1]) / (Vmax[1] - Vmin[1]);
    set_Lvl_Perc(lvl);


    //EC
    ecVoltage = volts[get_EC_Ch() + 3]; // Read the EC voltage
    

    //pH
    phVoltage = volts[get_PH_Ch() + 3]; // Read the PH voltage
    
    ph2(this);


    //Analog general
    float gen[2];
    uint8_t AnInGen_Ch[2];
    uint8_t tot = get_EC_Ch() + get_PH_Ch();
    uint8_t rnd = round((10 - tot) / 2);
    uint8_t ch1 = 10 - tot - rnd - 1;

    if (ch1 == get_EC_Ch())
        AnInGen_Ch[0] = ch1 - 1;
    else
        AnInGen_Ch[0] = ch1;

    uint8_t ch2 = 10 - tot - AnInGen_Ch[0];

    if (ch2 == get_PH_Ch())
        AnInGen_Ch[1] = ch2 + 1;
    else
        AnInGen_Ch[1] = ch2;

    gen[0] = volts[AnInGen_Ch[0] + 3];
    gen[1] = volts[AnInGen_Ch[1] + 3];
    set_Gen_Val(gen);
}

void ph1(Analog* analog)
{

	if (millis() - analog->now >= 1000) // 1000ms interval
	{
	    analog->now = millis();

		if (!analog->get_PH_Calibration())
		{
			analog->set_PH_Val(ph.readPH(analog->phVoltage, analog->get_WatTemp_Val())); // Convert voltage to PH with temperature compensation
		    analog->set_EC_Val(ec.readEC(analog->ecVoltage, analog->get_WatTemp_Val())); // Convert voltage to EC with temperature compensation
        }
	}

        if (analog->get_PH_Calibration() || strstr(analog->cmd, "PH") || strstr(analog->cmd, "EC"))
        {
            analog->set_PH_Calibration(1);
            
            if (strstr(analog->cmd, "PH"))
                ph.calibration(analog->phVoltage, analog->get_WatTemp_Val(), analog->cmd); // PH calibration process by Serial CMD
            
            if (strstr(analog->cmd, "EC"))
                ec.calibration(analog->ecVoltage, analog->get_WatTemp_Val(), analog->cmd); // EC calibration process by Serial CMD
        }

        if (strstr(analog->cmd, "EXITPH") || strstr(analog->cmd, "EXITEC"))
            analog->set_PH_Calibration(0);
}

#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
float pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

void ph2(Analog* analog)
{
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    
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
    // float temp = analog->get_WaterTemp_Val();
    // float T = temp + 273.15; // Kelvin cinsinden sıcaklık
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
    //         phVolt1 = analog->phVoltage + phFirst * R * T / (n * F) * log(10); // E0 = E + pH * R * T / (n * F) * ln10
    //     }
    //     else if (phVolt2 == 0)
    //     {
    //         phSecond = get_PH_Cal();
    //         // Nernst denklemi
    //         phVolt2 = analog->phVoltage + phSecond * R * T / (n * F) * log(10); // E0 = E + pH * R * T / (n * F) * ln10
    //     }

    //     set_PH_Calibration(0);
    // }

    // // Nernst denklemiyle pH hesaplama
    // float pHcalc = (analog->phVoltage - phVolt1) / (R * T / (n * F) * log(10)); // pH = (E - E0) / (R * T / (n * F) * ln10)
    // // Sıcaklık telafisi ekleme
    // float temperatureCoefficient = (R * T) / (n * F) * log(10);
    // float _phValue = phValue + (temp - 25.0) * temperatureCoefficient;

    float (*phCal)[2] = analog->get_PH_Cal();
    float phFirst = phCal[0][0], phSecond = phCal[1][0];
    float phVolt1 = phCal[0][1], phVolt2 = phCal[1][1];

    // Calculate the slope between two points
    float slope = (phSecond - phFirst) / (phVolt2 - phVolt1); // m = (y2 - y1) / (x2 - x1)
    // Calculate the y-intercept
    float intercept = phSecond - slope * phVolt2; // b = y1 - m * x1 | b = y2 - m * x2
    // Verilen voltaj için pH değerini hesaplama
    float phValue =  slope * (analog->phVoltage - phVolt1) + intercept; // y = m * x + b
    

    static float voltage;
    if (millis() - samplingTime > samplingInterval)
    {
        pHArray[pHArrayIndex++] = analog->phVoltage;
        if (pHArrayIndex == ArrayLenth)
            pHArrayIndex = 0;
        voltage = averageArray(pHArray, ArrayLenth);
        // phValue = 3.5 * voltage + analog->get_PH_Cal();
        samplingTime = millis();
    }
    if (millis() - printTime > 1000)
    {
        analog->set_PH_Val(phValue);
        ESP_LOGI(TAG,"ph = %f", ph);
        // ESP_LOGI(TAG,"pH = %f", pH);
        // ESP_LOGI(TAG,"voltage = %f", analog->phVoltage);
        printTime = millis();
    }
}
double averageArray(float* arr, int number)
{
    int i;
    float max, min;
    double avg;
    double amount = 0.0;
    
    if (number <= 0)
        return 0;

    if (number < 5)
    { // less than 5, calculated directly statistics
        for (i = 0; i < number; i++)
            amount += arr[i];
        avg = amount / number;
        return avg;
    }
    else
    {
        if (arr[0] < arr[1])
        {
        min = arr[0];
        max = arr[1];
        }
        else
        {
        min = arr[1];
        max = arr[0];
        }

        for (i = 2; i < number; i++)
        {
            if (arr[i] < min)
            {
                amount += min; // arr[i] < min
                min = arr[i];
            }
            else if (arr[i] > max)
            {
                amount += max; // arr[i] > max
                max = arr[i];
            }
            else
            {
                amount += arr[i]; // min <= arr[i] <= max
            }
        }
        avg = amount / (number - 2);
    }
    return avg;
}

}  // namespace water_quality
}  // namespace esphome