#include "myi2c.h"

namespace esphome {
namespace myi2c{

   float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

    Sensor *accel_x_sensor = new Sensor();
    Sensor *accel_y_sensor = new Sensor();
    Sensor *accel_z_sensor = new Sensor();
    Sensor *voltage_sensor = new Sensor();
    Sensor *percentage_sensor = new Sensor();
    Sensor *sayi = new Sensor();
    
    int sayac = 0;
    float x = 0.0;
    char *btname = "ESP32";

    int16_t adc[16];
    float volts[16], x, y, z, voltage, percentage;
    char *data = {};
    int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7, adc8, adc9, adc10, adc11, adc12, adc13, adc14, adc15;
    float volts0, volts1, volts2, volts3, volts4, volts5, volts6, volts7, volts8, volts9, volts10, volts11, volts12, volts13, volts14, volts15;

void Myi2c::device(char *name = "ESP32s")
{
    btname = name;
}

void Myi2c::gain(float g)
{
    x = g;
}

void Myi2c::setup()
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  bt
    
    SerialBT.begin(btname);
    ESP_LOGD("data", "Bluetooth is ready to pair\nDevice name: %s",btname);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  i2c
    
    Wire.begin();
    //Wire1.begin(SDA_1, SCL_1, freq_1);
    //I2C_1.begin(SDA_1, SCL_1, freq_1);
    //I2C_2.begin(SDA_2, SCL_2, freq_2);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
  
    //bool status1 = ads.begin(address1,&I2C);
    //bool status2 = ads2.begin(address2);
    //bool status3 = ads3.begin(address3);
    //bool status4 = ads4.begin(address4);
    bool status1 = ads1.begin(address1,&Wire);
    bool status2 = ads2.begin(address2,&Wire);
    bool status3 = ads3.begin(address3,&Wire);
    bool status4 = ads4.begin(address4,&Wire);
    if (!status1)
    {
      SerialBT.println("Failed to initialize ADS1115_1.");
      while (1);
    }
    if (!status2)
    {
      SerialBT.println("Failed to initialize ADS1115_2.");
      while (1);
    }
    if (!status3)
    {
      SerialBT.println("Failed to initialize ADS1115_3.");
      while (1);
    }
    if (!status4)
    {
      SerialBT.println("Failed to initialize ADS1115_4.");
      while (1);
    }

    // The ADC input range (or gain) can be changed via the following
    // functions, but be careful never to exceed VDD +0.3V max, or to
    // exceed the upper and lower limits if you adjust the input range!
    // Setting these values incorrectly may destroy your ADC!
    //                                                                ADS1015  ADS1115
    //                                                                -------  -------
    // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    ads1.setGain(GAIN_ONE); 
    ads2.setGain(GAIN_ONE); 
    ads3.setGain(GAIN_ONE);
    ads4.setGain(GAIN_ONE);
    // ads.setDataRate(RATE_ADS1115_860SPS);
    ads1.setDataRate(RATE_ADS1115_860SPS);
    ads2.setDataRate(RATE_ADS1115_860SPS);
    ads3.setDataRate(RATE_ADS1115_860SPS);
    ads4.setDataRate(RATE_ADS1115_860SPS);
    // ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1,true);
    ads1.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1,true);
    ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1,true);
    ads3.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1,true);
    ads4.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1,true);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADXL345

    bool status5 = accel.begin(ADXL345_DEFAULT_ADDRESS);

    if (!status5)
    {
      Serial.println("Failed to initialize ADXL345.");
      while (1);
    }

    /* Set the range to whatever is appropriate for your project */
    //accel.setRange(ADXL345_RANGE_16_G);  ///< +/- 16g
    //accel.setRange(ADXL345_RANGE_8_G);  ///< +/- 8g
    //accel.setRange(ADXL345_RANGE_4_G);  ///< +/- 4g
    accel.setRange(ADXL345_RANGE_2_G);  ///< +/- 2g (default value)
    
    //accel.setDataRate(ADXL345_DATARATE_3200_HZ);  ///< 1600Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_1600_HZ);  ///<  800Hz Bandwidth    90�A IDD
    //accel.setDataRate(ADXL345_DATARATE_800_HZ);  ///<  400Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_400_HZ);  ///<  200Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_200_HZ);  ///<  100Hz Bandwidth   140�A IDD
    accel.setDataRate(ADXL345_DATARATE_100_HZ);  ///<   50Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_50_HZ);   ///<   25Hz Bandwidth    90�A IDD
    //accel.setDataRate(ADXL345_DATARATE_25_HZ);   ///< 12.5Hz Bandwidth    60�A IDD
    //accel.setDataRate(ADXL345_DATARATE_12_5_HZ); ///< 6.25Hz Bandwidth    50�A IDD
    //accel.setDataRate(ADXL345_DATARATE_6_25HZ);  ///< 3.13Hz Bandwidth    45�A IDD
    //accel.setDataRate(ADXL345_DATARATE_3_13_HZ); ///< 1.56Hz Bandwidth    40�A IDD
    //accel.setDataRate(ADXL345_DATARATE_1_56_HZ); ///< 0.78Hz Bandwidth    34�A IDD
    //accel.setDataRate(ADXL345_DATARATE_0_78_HZ); ///< 0.39Hz Bandwidth    23�A IDD
    //accel.setDataRate(ADXL345_DATARATE_0_39_HZ); ///< 0.20Hz Bandwidth    23�A IDD
    //accel.setDataRate(ADXL345_DATARATE_0_20_HZ); ///< 0.10Hz Bandwidth    23�A IDD
    //accel.setDataRate(ADXL345_DATARATE_0_10_HZ); ///< 0.05Hz Bandwidth    23�A IDD (default value)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAX17048

    bool status6 = maxlipo.begin(&Wire);

    if (!status6)
    {
      Serial.println("Failed to initialize MAX17048.");
      while (1);
    }
}

void Myi2c::loop()
{
    // for(int i=0;i<4;i++)
    // {
    //   adc[i] = ads1.readADC_SingleEnded(i);
    //   volts[i] = ads1.computeVolts(adc[i]);
    //   data = data + String(volts[i]) + ",";
    // }
    // for(int i=4;i<8;i++)
    // {
    //   adc[i] = ads2.readADC_SingleEnded(i);
    //   volts[i] = ads2.computeVolts(adc[i]);
    //   data = data + String(volts[i]) + ",";
    // }
    // for(int i=8;i<12;i++)
    // {
    //   adc[i] = ads3.readADC_SingleEnded(i);
    //   volts[i] = ads3.computeVolts(adc[i]);
    //   data = data + String(volts[i]) + ",";
    // }
    // for(int i=12;i<16;i++)
    // {
    //   adc[i] = ads4.readADC_SingleEnded(i);
    //   volts[i] = ads4.computeVolts(adc[i]);
    //   data = data + String(volts[i]) + ",";
    // }

    // x = accel.getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    // y = accel.getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    // z = accel.getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    // voltage = maxlipo.cellVoltage();
    // percentage = maxlipo.cellPercent();
    
    // data = data + String(x) + "," + String(y) + "," + String(z) + "," + String(voltage) + "," + String(percentage);

    // SerialBT.println(data);
 
    adc0 = ads1.readADC_SingleEnded(0);
    adc1 = ads1.readADC_SingleEnded(1);
    adc2 = ads1.readADC_SingleEnded(2);
    adc3 = ads1.readADC_SingleEnded(3);
    adc4 = ads2.readADC_SingleEnded(0);
    adc5 = ads2.readADC_SingleEnded(1);
    adc6 = ads2.readADC_SingleEnded(2);
    adc7 = ads2.readADC_SingleEnded(3);
    adc8 = ads3.readADC_SingleEnded(0);
    adc9 = ads3.readADC_SingleEnded(1);
    adc10 = ads3.readADC_SingleEnded(2);
    adc11 = ads3.readADC_SingleEnded(3);
    adc12 = ads4.readADC_SingleEnded(0);
    adc13 = ads4.readADC_SingleEnded(1);
    adc14 = ads4.readADC_SingleEnded(2);
    adc15 = ads4.readADC_SingleEnded(3);
 
    volts0 = ads1.computeVolts(adc0) * x;
    volts1 = ads1.computeVolts(adc1) * x;
    volts2 = ads1.computeVolts(adc2) * x;
    volts3 = ads1.computeVolts(adc3) * x;
    volts4 = ads2.computeVolts(adc4) * x;
    volts5 = ads2.computeVolts(adc5) * x;
    volts6 = ads2.computeVolts(adc6) * x;
    volts7 = ads2.computeVolts(adc7) * x;
    volts8 = ads3.computeVolts(adc8) * x;
    volts9 = ads3.computeVolts(adc9) * x;
    volts10 = ads3.computeVolts(adc10) * x;
    volts11 = ads3.computeVolts(adc11) * x;
    volts12 = ads4.computeVolts(adc12) * x;
    volts13 = ads4.computeVolts(adc13) * x;
    volts14 = ads4.computeVolts(adc14) * x;
    volts15 = ads4.computeVolts(adc15) * x;

      SerialBT.println(String(volts0)+","+
                       String(volts1)+","+
                       String(volts2)+","+
                       String(volts3)+","+
                       String(volts4)+","+
                       String(volts5)+","+
                       String(volts6)+","+
                       String(volts7)+","+
                       String(volts8)+","+
                       String(volts9)+","+
                       String(volts10)+","+
                       String(volts11)+","+
                       String(volts12)+","+
                       String(volts13)+","+
                       String(volts14)+","+
                       String(volts15)+","+
                       String(x)+","+
                       String(y)+","+
                       String(z)+","+
                       String(percentage));
    sayac += 1;

    // accel_x_sensor->publish_state(x);
    // accel_y_sensor->publish_state(y);
    // accel_z_sensor->publish_state(z);
    
    // voltage_sensor->publish_state(voltage);
    // percentage_sensor->publish_state(percentage);
    sayi->publish_state(sayac);
}

} //namespace myi2c
} //namespace esphome
