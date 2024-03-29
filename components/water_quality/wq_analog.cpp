#include "wq_analog.h"

namespace esphome {
namespace water_quality {

// WaterQuality my;

void Analog::Analog_Input_Driver(float volts[])
{
    // my.ADS1115_Driver(volts);
    float WT_Res = (volts[0] * 1000.0) / (5.0 - volts[0]) * (get_WTemp_Res() / 1000.0); //R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
    if (WT_Res > 3904.8) //max temp limit and set model multiplier 
        WT_Res = 3904.8 * (get_WTemp_Res() / 1000.0);
    else
        WT_Res = WT_Res * (get_WTemp_Res() / 1000.0);
    float WT = (sqrt((-0.00232 * WT_Res) + 17.59246) - 3.908) / (-0.00116); //Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116
    set_WTemp_Val(WT);
    
    set_VPow_Val(volts[1] * 6); //Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k
    
    float lvl[2], Vmin[2], Vmax[2];
    uint16_t res, volt, *resmin = get_ResMin(), *resmax = get_ResMax();
    if (get_version() == 0) { res = 1000; volt = get_VPow_Val(); }
    if (get_version() == 1) { res = 270; volt = 5; }
    Vmin[0] = volt * resmin[0] / (res + resmin[0]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    Vmin[1] = volt * resmin[1] / (res + resmin[1]);
    Vmax[0] = volt * resmax[0] / (res + resmax[0]);
    Vmax[1] = volt * resmax[1] / (res + resmax[1]);
    lvl[0] = 100 * (volts[2] - Vmin[0]) / (Vmax[0] - Vmin[0]);
    lvl[1] = 100 * (volts[3] - Vmin[1]) / (Vmax[1] - Vmin[1]);
    set_Lvl_Perc(lvl);

    set_EC_Val(volts[get_EC_Ch() + 3]);
    set_PH_Val(volts[get_PH_Ch() + 3]);

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

bool calibrationIsRunning = false;
// void Analog::ec_ph()
// {
// 	unsigned long now = millis();
// 	if (now - last[0] >= intervals[0]) //1000ms interval
// 	{
// 		last[0] = now;
// 		if (calibrationIsRunning)
// 		{
// 			Serial.println(F("[main]...>>>>>> calibration is running, to exit send exitph or exitec through serial <<<<<<"));
// 			//water temperature
// 			temperature = getWaterTemperature();
// 			//EC
// 			// ecVoltage = ads2.readADC_SingleEnded(3) / 10;
// 			Serial.print(F("[EC Voltage]... ecVoltage: "));
// 			Serial.println(ecVoltage);
// 			EC = ec.readEC(ecVoltage, temperature); // convert voltage to EC with temperature compensation
// 			Serial.print(F("[EC Read]... EC: "));
// 			Serial.print(EC);
// 			Serial.println(F("ms/cm"));
// 			//pH
// 			// phVoltage = ads2.readADC_SingleEnded(1) / 10;
// 			Serial.print(F("[pH Voltage]... phVoltage: "));
// 			Serial.println(phVoltage);
// 			PH = ph.readPH(phVoltage, temperature);
// 			Serial.print(F("[pH Read]... pH: "));
// 			Serial.println(PH);
// 		}

// 		if (readSerial(cmd))
// 		{
// 			strupr(cmd);
// 			if (calibrationIsRunning || strstr(cmd, "PH") || strstr(cmd, "EC"))
// 			{
// 				calibrationIsRunning = true;
// 				Serial.println(F("[]... >>>>>calibration is now running PH and EC are both reading, if you want to stop this process enter EXITPH or EXITEC in Serial Monitor<<<<<"));
// 				if (strstr(cmd, "PH"))
// 				{
// 					ph.calibration(phVoltage, temperature, cmd); //PH calibration process by Serail CMD
// 				}
// 				if (strstr(cmd, "EC"))
// 				{
// 					ec.calibration(ecVoltage, temperature, cmd); //EC calibration process by Serail CMD
// 				}
// 			}
// 			if (strstr(cmd, "EXITPH") || strstr(cmd, "EXITEC"))
// 			{
// 				calibrationIsRunning = false;
// 			}
// 		}
// 	}
// 	if (now - last[3] >= intervals[3]) //5000ms interval
// 	{
// 		last[3] = now; 
// 		if (!calibrationIsRunning)
// 		{
// 			temperature = getWaterTemperature(); // read your temperature sensor to execute temperature compensation
// 			Serial.print("temperature:");
// 			Serial.print(temperature, 1);
// 			Serial.println("^C");

// 			// ecVoltage = ads2.readADC_SingleEnded(3) / 10;
// 			Serial.print("ecVoltage:");
// 			Serial.println(ecVoltage, 4);

// 			EC = ec.readEC(ecVoltage, temperature); // convert voltage to EC with temperature compensation
// 			Serial.print("EC:");
// 			Serial.print(EC, 4);
// 			Serial.println("ms/cm");

// 			// phVoltage = ads2.readADC_SingleEnded(1) / 10; // read the voltage
// 			Serial.print("phVoltage:");
// 			Serial.println(phVoltage, 4);
// 			PH = ph.readPH(phVoltage, temperature); // convert voltage to pH with temperature compensation
// 			Serial.print("pH:");
// 			Serial.println(PH, 4);
// 		}
// 	}
// }

// void Analog::ec_ph2()
// {
//     if(millis()-timepoint>1000U)                             //time interval: 1s
//     {
//         timepoint = millis();
//         //temperature = readTemperature();                   // read your temperature sensor to execute temperature compensation
//         // voltagePH = analogRead(PH_PIN)/1024.0*5000;          // read the ph voltage
//         // PH    = ph.readPH(voltagePH,WT);       // convert voltage to pH with temperature compensation
//         Serial.print("pH:");
//         Serial.print(PH,2);
//         // voltageEC = analogRead(EC_PIN)/1024.0*5000;
//         // EC    = ec.readEC(voltageEC,WT);       // convert voltage to EC with temperature compensation
//         Serial.print(", EC:");
//         Serial.print(EC,2);
//         Serial.println("ms/cm");
//     }
//     if(readSerial(cmd)){
//         strupr(cmd);
//         if(strstr(cmd,"PH")){
//             ph.calibration(voltagePH,WT,cmd);       //PH calibration process by Serail CMD
//         }
//         if(strstr(cmd,"EC")){
//             ec.calibration(voltageEC,WT,cmd);       //EC calibration process by Serail CMD
//         }
//     }
// }

// int i = 0;
// bool readSerial(char result[])
// {
// 	while (Serial.available() > 0)
// 	{
// 		char inChar = Serial.read();
// 		if (inChar == '\n')
// 		{
// 			result[i] = '\0';
// 			Serial.flush();
// 			i = 0;
// 			return true;
// 		}
// 		if (inChar != '\r')
// 		{
// 			result[i] = inChar;
// 			i++;
// 		}
// 		delay(1);
// 	}
// 	return false;
// }

}  // namespace water_quality
}  // namespace esphome