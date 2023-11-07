#include "mux.h"
#include "analog.h"
#include "water_quality.h"

namespace esphome {
namespace water_quality {

void ADS1115::Analog_Input_Driver()
{
    top = AnInEC_Ch + AnInEC_Ch;
    rnd = round((6 - top) / 2);
    AnInGen_Ch[0] = 6 - top - rnd - 1;
    AnInGen_Ch[1] = 6 - top - AnInGen_Ch[0];
    AnInGen_Ch[0] = (AnInGen_Ch[0] == AnInEC_Ch)? AnInGen_Ch[0] - 1 : AnInGen_Ch[0];
    AnInGen_Ch[1] = (AnInGen_Ch[1] == AnInEC_Ch)? AnInGen_Ch[1] + 1 : AnInGen_Ch[1];

        ESP_LOGD(analog,"ads = %f", volts[3+4]);
        ESP_LOGD(analog,"ads = %f", (ads2.readADC_SingleEnded(3)/10));
    VPow = (float)volts[1] * 6; //Vin = Vout * (R1 + R2) / R2; R1 = 10k, R2 = 2k
    LvlPerc[0] = (float)volts[2] * 100 / 5 * AnInLvl_ResMax[0] / (1000 + AnInLvl_ResMax[0]) - 5 * AnInLvl_ResMin[0] / (1000 + AnInLvl_ResMin[0]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k
    LvlPerc[1] = (float)volts[3] * 100 / 5 * AnInLvl_ResMax[1] / (1000 + AnInLvl_ResMax[1]) - 5 * AnInLvl_ResMin[1] / (1000 + AnInLvl_ResMin[1]); //Vout = Vin * R2 / (R1 + R2); R1 = 10k

    ecValue = volts[AnInEC_Ch];
    phValue = volts[AnInPH_Ch];
    AnGen[0] = volts[AnInGen_Ch[0] + 4];
    AnGen[1] = volts[AnInGen_Ch[1] + 4];
}

void ADS1115::void ec_ph()
{
	unsigned long now = millis();
	if (now - last[0] >= intervals[0]) //1000ms interval
	{
		last[0] = now;
		if (calibrationIsRunning)
		{
			Serial.println(F("[main]...>>>>>> calibration is running, to exit send exitph or exitec through serial <<<<<<"));
			//water temperature
			temperature = getWaterTemperature();
			//EC
			ecVoltage = ads2.readADC_SingleEnded(3) / 10;
			Serial.print(F("[EC Voltage]... ecVoltage: "));
			Serial.println(ecVoltage);
			ecValue = ec.readEC(ecVoltage, temperature); // convert voltage to EC with temperature compensation
			Serial.print(F("[EC Read]... EC: "));
			Serial.print(ecValue);
			Serial.println(F("ms/cm"));
			//pH
			phVoltage = ads2.readADC_SingleEnded(1) / 10;
			Serial.print(F("[pH Voltage]... phVoltage: "));
			Serial.println(phVoltage);
			phValue = ph.readPH(phVoltage, temperature);
			Serial.print(F("[pH Read]... pH: "));
			Serial.println(phValue);
		}

		if (readSerial(cmd))
		{
			strupr(cmd);
			if (calibrationIsRunning || strstr(cmd, "PH") || strstr(cmd, "EC"))
			{
				calibrationIsRunning = true;
				Serial.println(F("[]... >>>>>calibration is now running PH and EC are both reading, if you want to stop this process enter EXITPH or EXITEC in Serial Monitor<<<<<"));
				if (strstr(cmd, "PH"))
				{
					ph.calibration(phVoltage, temperature, cmd); //PH calibration process by Serail CMD
				}
				if (strstr(cmd, "EC"))
				{
					ec.calibration(ecVoltage, temperature, cmd); //EC calibration process by Serail CMD
				}
			}
			if (strstr(cmd, "EXITPH") || strstr(cmd, "EXITEC"))
			{
				calibrationIsRunning = false;
			}
		}
	}
	if (now - last[3] >= intervals[3]) //5000ms interval
	{
		last[3] = now; 
		if (!calibrationIsRunning)
		{
			temperature = getWaterTemperature(); // read your temperature sensor to execute temperature compensation
			Serial.print("temperature:");
			Serial.print(temperature, 1);
			Serial.println("^C");

			ecVoltage = ads2.readADC_SingleEnded(3) / 10;
			Serial.print("ecVoltage:");
			Serial.println(ecVoltage, 4);

			ecValue = ec.readEC(ecVoltage, temperature); // convert voltage to EC with temperature compensation
			Serial.print("EC:");
			Serial.print(ecValue, 4);
			Serial.println("ms/cm");

			phVoltage = ads2.readADC_SingleEnded(1) / 10; // read the voltage
			Serial.print("phVoltage:");
			Serial.println(phVoltage, 4);
			phValue = ph.readPH(phVoltage, temperature); // convert voltage to pH with temperature compensation
			Serial.print("pH:");
			Serial.println(phValue, 4);
		}
	}
}

void ec_ph2()
{
    if(millis()-timepoint>1000U)                             //time interval: 1s
    {
        timepoint = millis();
        //temperature = readTemperature();                   // read your temperature sensor to execute temperature compensation
        // voltagePH = analogRead(PH_PIN)/1024.0*5000;          // read the ph voltage
        // phValue    = ph.readPH(voltagePH,WT);       // convert voltage to pH with temperature compensation
        Serial.print("pH:");
        Serial.print(phValue,2);
        // voltageEC = analogRead(EC_PIN)/1024.0*5000;
        // ecValue    = ec.readEC(voltageEC,WT);       // convert voltage to EC with temperature compensation
        Serial.print(", EC:");
        Serial.print(ecValue,2);
        Serial.println("ms/cm");
    }
    if(readSerial(cmd)){
        strupr(cmd);
        if(strstr(cmd,"PH")){
            ph.calibration(voltagePH,WT,cmd);       //PH calibration process by Serail CMD
        }
        if(strstr(cmd,"EC")){
            ec.calibration(voltageEC,WT,cmd);       //EC calibration process by Serail CMD
        }
    }
}


}  // namespace water_quality
}  // namespace esphome