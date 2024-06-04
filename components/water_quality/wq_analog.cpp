#include "wq_analog.h"

namespace esphome {
namespace water_quality {

// WaterQuality my;

    DFRobot_PH ph;
    DFRobot_EC ec;

void ph1(Analog* analog)
{

	if (millis() - analog->now >= 1000) // 1000ms interval
	{
	    analog->now = millis();

		if (!analog->get_PH_Calibration())
		{
			analog->set_PH_Val(ph.readPH(analog->phVoltage, analog->get_WaterTemp_Val())); // Convert voltage to PH with temperature compensation
		    analog->set_EC_Val(ec.readEC(analog->ecVoltage, analog->get_WaterTemp_Val())); // Convert voltage to EC with temperature compensation
        }
	}

        if (analog->get_PH_Calibration() || strstr(analog->cmd, "PH") || strstr(analog->cmd, "EC"))
        {
            analog->set_PH_Calibration(1);
            
            if (strstr(analog->cmd, "PH"))
                ph.calibration(analog->phVoltage, analog->get_WaterTemp_Val(), analog->cmd); // PH calibration process by Serial CMD
            
            if (strstr(analog->cmd, "EC"))
                ec.calibration(analog->ecVoltage, analog->get_WaterTemp_Val(), analog->cmd); // EC calibration process by Serial CMD
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
    // Sıcaklık telafisi için Nernst sabiti
    float T = analog->get_WaterTemp_Val() + 273.15; // Kelvin cinsinden sıcaklık
    const float R = 8.314; // Gaz sabiti, J/(mol*K)
    const float F = 96485; // Faraday sabiti, C/mol
    const float n = 1; // Elektron sayısı (pH ölçümünde genellikle 1)
    float k = (R * T) / F * 1000; // mV başına değişim (2.303 * R * T / F)

    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float phValue = 0;

    float phStandard = 7;
    float E0 = 1.5;
    // Voltajı pH'a dönüştürmek için Nernst denklemi
    // float ph = - (analog->phVoltage / ((R * T) / (n * F) * log(10)));
    float ph = phStandard + ((R * T) / (n * F)) * log(10) * log10(analog->phVoltage);
    // Nernst denklemiyle pH hesaplama
    float pH = phStandard + (analog->phVoltage - E0) / ((R * T) / (n * F) * log(10));

    if (millis() - samplingTime > samplingInterval)
    {
        phValue = 3.5 * analog->phVoltage + analog->get_PH_Cal();
        samplingTime = millis();
    }
    if (millis() - printTime > 1000)
    {
        analog->set_PH_Val(phValue);
        ESP_LOGI(TAG,"ph = %f", ph);
        ESP_LOGI(TAG,"pH = %f", pH);
        printTime = millis();
    }
}

void Analog::Analog_Input_Driver(float volts[])
{
    // my.ADS1115_Driver(volts);

    //Water Temperature
    uint16_t timeperiod = 1000; // Wait time before each update

    if (millis() - get_Analog_Timepoint() >= timeperiod)
    {
        float model_multiply = get_WaterTemp_Res() / 1000.0; // Multiplier of the resistance of the temperature sensor model relative to the pt1000 sensor
        float WaterTemp_Res = (volts[0] * 1000.0) / (5.0 - volts[0]) * model_multiply; // R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
        
        if (WaterTemp_Res > 3904.8) // Max temp limit and set model multiplier 
            WaterTemp_Res = 3904.8 * model_multiply;
        else
            WaterTemp_Res = WaterTemp_Res * model_multiply;
        
        float WaterTemp = (sqrt((-0.00232 * WaterTemp_Res) + 17.59246) - 3.908) / (-0.00116); // Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116
        
        set_WaterTemp_Val(WaterTemp);
        set_Analog_Timepoint(millis());
    }
    

    //Power
    set_VoltagePow_Val(volts[1] * 6); // Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k
    

    //Level
    float lvl[2], Vmin[2], Vmax[2];
    uint16_t res, volt, *resmin = get_ResMin(), *resmax = get_ResMax();

    if (get_version() == 0) { res = 1000; volt = get_VoltagePow_Val(); } // Version check
    if (get_version() == 1) { res = 270; volt = 5; }

    Vmin[0] = volt * resmin[0] / (res + resmin[0]); // Vout = Vin * R2 / (R1 + R2); R1 = 10k
    Vmin[1] = volt * resmin[1] / (res + resmin[1]);
    Vmax[0] = volt * resmax[0] / (res + resmax[0]);
    Vmax[1] = volt * resmax[1] / (res + resmax[1]);

    lvl[0] = 100 * (volts[2] - Vmin[0]) / (Vmax[0] - Vmin[0]);
    lvl[1] = 100 * (volts[3] - Vmin[1]) / (Vmax[1] - Vmin[1]);
    set_Level_Perc(lvl);


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

}  // namespace water_quality
}  // namespace esphome