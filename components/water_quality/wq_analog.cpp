#include "wq_analog.h"

namespace esphome {
namespace water_quality {

// WaterQuality my;

DFRobot_EC ec;
DFRobot_PH ph;

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
    
    ec_ph();


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

void Analog::ec_ph()
{
    
	if (millis() - now >= 1000) // 1000ms interval
	{
	    now = millis();

		if (!get_EC_PH_Calibration())
		{
			set_EC_Val(ec.readEC(ecVoltage, get_WaterTemp_Val())); // Convert voltage to EC with temperature compensation
			set_PH_Val(ph.readPH(phVoltage, get_WaterTemp_Val())); // Convert voltage to PH with temperature compensation
		}

        if (get_EC_PH_Calibration() || strstr(cmd, "PH") || strstr(cmd, "EC"))
        {
            set_EC_PH_Calibration(1);
            
            if (strstr(cmd, "PH"))
                ph.calibration(phVoltage, get_WTemp_Val(), cmd); // PH calibration process by Serial CMD
            
            if (strstr(cmd, "EC"))
                ec.calibration(ecVoltage, get_WTemp_Val(), cmd); // EC calibration process by Serial CMD
        }

        if (strstr(cmd, "EXITPH") || strstr(cmd, "EXITEC"))
            set_EC_PH_Calibration(0);
	}
}

void Analog::ec_ph2()
{
    if(millis() - now > 1000U)                             //time interval: 1s
    {
        now = millis();

        set_EC_Val(ec.readEC(ecVoltage, get_WTemp_Val()));       // convert voltage to EC with temperature compensation
        
        set_PH_Val(ph.readPH(phVoltage, get_WTemp_Val()));       // convert voltage to pH with temperature compensation
    }

    if(readSerial(cmd))
    {
        strupr(cmd);
        if(strstr(cmd,"PH")){
            ph.calibration(voltagePH,get_WTemp_Val(),cmd);       //PH calibration process by Serail CMD
        }
        if(strstr(cmd,"EC")){
            ec.calibration(voltageEC,get_WTemp_Val(),cmd);       //EC calibration process by Serail CMD
        }
    }
}

}  // namespace water_quality
}  // namespace esphome