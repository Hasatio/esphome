#include "myi2c.h"

extern "C" {
uint8_t temprature_sens_read();
}

namespace esphome {
namespace myi2c {
    
static const char *TAG = "myi2c.sensor";

    BluetoothSerial SerialBT; // bluetooth yeni adlandırması
    
    #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error Bluetooth off--Run `make menuconfig` to enable it 
    #endif
    
    //Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
    Adafruit_ADS1115 ads1; // ads1115 yeni adlandırması
    Adafruit_ADS1115 ads2;
    Adafruit_ADS1115 ads3;
    Adafruit_ADS1115 ads4;
    //Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
    
    Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // adx345 ayarı
    
    Adafruit_MAX17048 maxlipo; // max17048 yeni adlandırması

    // i2c ayarları
    #define SDA 21 
    #define SCL 22
    #define freq 800000
    /*
    #define SDA_1 32
    #define SCL_1 33
    #define freq_1 800000
    
    #define SDA_2 25
    #define SCL_2 26
    #define freq_2 800000
    
    #define SDA_3 27
    #define SCL_3 14
    #define freq_3 800000
    
    #define SDA_4 9
    #define SCL_4 10
    #define freq_4 800000
    */
    // ads1115 i2c adres ayarları
    #define address1 0x48
    #define address2 0x49
    #define address3 0x4a
    #define address4 0x4b
    
    // TwoWire I2C_1 = TwoWire(0);
    // TwoWire I2C_2 = TwoWire(1);

    // Sensor *accel_x_sensor = new Sensor();
    // Sensor *accel_y_sensor = new Sensor();
    // Sensor *accel_z_sensor = new Sensor();
    // Sensor *voltage_sensor = new Sensor();
    // Sensor *percentage_sensor = new Sensor();
    // sensor *sayi = new sensor();
    
    String btname = "ESP32"; // bt standart adı

    uint16_t adc[16];
    uint64_t sayac = 0;
    float volts[16], x, y, z, voltage, percentage, mygain = 1.0, temperature = NAN;
    double adxlmultiplier;
    String data = "";
    bool success = false;

void Myi2c::bluetooth(String b) // bluetooth fonksiyonu
{
    btname = b;
}

void Myi2c::gain(float g) // kazanç fonksiyonu
{
    mygain = g;
}

void Myi2c::setup() // ayar fonksiyonu
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Bluetooth
    
    SerialBT.begin(btname);
    ESP_LOGD("data", "Bluetooth is ready to pair\nDevice name: %s",btname);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  i2c
    
    Wire.begin(SDA,SCL,freq);
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
    
void Myi2c::loop() // döngü fonksiyonu
{  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  İnternal Temp
    
  uint8_t raw = temprature_sens_read();
  ESP_LOGV(TAG, "Raw temperature value: %d", raw);
  temperature = (raw - 32) / 1.8f;
  success = (raw != 128);
    
  // if (success && std::isfinite(temperature)) {
  //   this->publish_state(temperature);
  // } else {
  //   ESP_LOGD(TAG, "Ignoring invalid temperature (success=%d, value=%.1f)", success, temperature);
  //   if (!this->has_state()) {
  //     this->publish_state(NAN);
  //   }
  // }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
    
    for(int i=0;i<4;i++)
    {
      adc[i] = ads1.readADC_SingleEnded(i%4);
      volts[i] = ads1.computeVolts(adc[i]) * mygain;
      data = data + String(volts[i]) + ",";
    }
    for(int i=4;i<8;i++)
    {
      adc[i] = ads2.readADC_SingleEnded(i%4);
      volts[i] = ads2.computeVolts(adc[i]) * mygain;
      data = data + String(volts[i]) + ",";
    }
    for(int i=8;i<12;i++)
    {
      adc[i] = ads3.readADC_SingleEnded(i%4);
      volts[i] = ads3.computeVolts(adc[i]) * mygain;
      data = data + String(volts[i]) + ",";
    }
    for(int i=12;i<16;i++)
    {
      adc[i] = ads4.readADC_SingleEnded(i%4);
      volts[i] = ads4.computeVolts(adc[i]) * mygain;
      data = data + String(volts[i]) + ",";
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADXL345
    
    adxlmultiplier = ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    x = accel.getX() * adxlmultiplier;
    y = accel.getY() * adxlmultiplier;
    z = accel.getZ() * adxlmultiplier;
  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAX17048
    
    voltage = maxlipo.cellVoltage();
    percentage = maxlipo.cellPercent();
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Bluetooth
    
    data = data + String(x) + "," + String(y) + "," + String(z) + "," + String(voltage) + "," + String(percentage) + "," + String(temperature);
    
    SerialBT.println(data);
    data = "";
 
    sayac += 1;
    
    if(SerialBT.available())
      {
        mygain = float(SerialBT.read());
        ESP_LOGD("data", "data: %f",mygain);
      }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Sensor
    
#ifdef USE_SENSOR
if (this->sample_ != nullptr) this->sample_->publish_state(sayac);
if (this->sample_sec_ != nullptr) this->sample_sec_->publish_state(sayac*1000/millis());
    // ESP_LOGD(TAG, "Sample = %d",sample_);
#endif
}

void Myi2c::update()
{ 
}

} //namespace myi2c
} //namespace esphome
