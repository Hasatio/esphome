#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_analog.h"

namespace esphome {
namespace water_quality {

WaterQuality my;

void Analog::Analog_Input_Driver(float volts[])
{
    // my.ADS1115_Driver(volts);
	float WT_Res = (float)(volts[1] * 1000) / (5 - volts[1]) * (get_WT_Res() / 1000); //R2 = (Vout * R1) / (Vin - Vout); Vin = 5V, R1 = 1k
    set_WT_Val((float)(sqrt((-0.00232 * WT_Res) + 17.59246) - 3.908) / (-0.00116)); //Temp = (√(-0,00232 * R + 17,59246) - 3,908) / -0,00116

    set_VPow_Val((float)volts[2] * 6); //Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k
    
	float lvl[2];
    lvl[0] = (float)volts[3] * 100 / 5 * AnInLvl_ResMax[0] / (1000 + AnInLvl_ResMax[0]) - 5 * AnInLvl_ResMin[0] / (1000 + AnInLvl_ResMin[0]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    lvl[1] = (float)volts[0] * 100 / 5 * AnInLvl_ResMax[1] / (1000 + AnInLvl_ResMax[1]) - 5 * AnInLvl_ResMin[1] / (1000 + AnInLvl_ResMin[1]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    set_Lvl_Perc(lvl);
	
    set_EC_Val(volts[(AnInEC_Ch > 7 ? 4 : AnInEC_Ch)]);
    set_PH_Val(volts[(AnInPH_Ch > 7 ? 4 : AnInPH_Ch)]);
        ESP_LOGD(TAG,"t1 = %f", get_EC_Val());
        ESP_LOGD(TAG,"c1 = %d", (AnInEC_Ch > 7 ? 4 : AnInEC_Ch));
        ESP_LOGD(TAG,"t2 = %f", get_PH_Val());
        ESP_LOGD(TAG,"c2 = %d", (AnInPH_Ch > 7 ? 4 : AnInPH_Ch));
        // ESP_LOGD(TAG,"ads = %f", volts[3+4]);
        // ESP_LOGD(TAG,"ads1 = %f", (ads2.readADC_SingleEnded(3)/10));

    uint8_t tot, rnd;
    tot = AnInEC_Ch + AnInPH_Ch;
    rnd = round((10 - tot) / 2);
    AnInGen_Ch[0] = 4 + (((10 - tot - rnd - 1) == AnInEC_Ch) ? 10 - tot - rnd - 2 : 10 - tot - rnd - 1);
    AnInGen_Ch[1] = 4 + (((10 - tot - AnInGen_Ch[0]) == AnInPH_Ch) ? 10 - tot - AnInGen_Ch[0] + 1 : 10 - tot - AnInGen_Ch[0]);
    AnInGen_Val[0] = volts[((AnInGen_Ch[0] > 7) ? 4 : AnInGen_Ch[0])];
        ESP_LOGD(TAG,"t3 = %f", AnInGen_Val[0]);
        ESP_LOGD(TAG,"c3 = %d", (AnInGen_Ch[0] > 7 ? 4 : AnInGen_Ch[0]));
    AnInGen_Val[1] = volts[((AnInGen_Ch[1] > 7) ? 4 : AnInGen_Ch[1])];
        ESP_LOGD(TAG,"t4 = %f", AnInGen_Val[0]);
        ESP_LOGD(TAG,"c4 = %d", (AnInGen_Ch[1] > 7 ? 4 : AnInGen_Ch[1]));
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