#include "posture_analyzer.h"

namespace esphome {
namespace posture_analyzer {

// BluetoothSerial SerialBT;

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;
Adafruit_ADS1115 ads4;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

Adafruit_MAX17048 maxlipo;

UUID uuid;

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
BLEService *pService;
BLEAdvertising *pAdvertising;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  UUID
void Posture_Analyzer::uuid_set()
{
    uint32_t seed1 = random(999999999);
    uint32_t seed2 = random(999999999);
    uuid.seed(seed1, seed2);
    uuid.generate();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Bluetooth
void Posture_Analyzer::bt_set()
{
    BLEDevice::init(get_bluetooth_name().c_str());  

    pServer = BLEDevice::createServer();

    pService = pServer->createService(uuid.toCharArray());

    pCharacteristic = pService->createCharacteristic(
                                                    uuid.toCharArray(),
                                                    BLECharacteristic::PROPERTY_READ |
                                                    BLECharacteristic::PROPERTY_WRITE |
                                                    BLECharacteristic::PROPERTY_NOTIFY
                                                    );

    // pCharacteristic->setValue("Hello World");
    pService->start();

    pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    // // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    // pAdvertising->start();
    pAdvertising->addServiceUUID(uuid.toCharArray());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    // pAdvertising->setMinPreferred(0x12);
    // // pAdvertising->setScanResponse(false);
    // pAdvertising->setMinPreferred(0x00);
    BLEDevice::startAdvertising();

    // SerialBT.begin(btname.c_str());
}
void Posture_Analyzer::bt()
{
    // ESP_LOGI(TAG, "data: %s", data);
    
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();

    // SerialBT.println(data.c_str());
    data = "";

    sayac += 1;
    
    // if(SerialBT.available())
    //   {
    //     mygain = float(SerialBT.read());
    //     ESP_LOGD(TAG, "data: %f", mygain);
    //   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  i2c
void Posture_Analyzer::i2c_set()
{
    Wire.begin(SDA,SCL,frq);
    //Wire1.begin(SDA_1, SCL_1, freq_1);
    //I2C_1.begin(SDA_1, SCL_1, freq_1);
    //I2C_2.begin(SDA_2, SCL_2, freq_2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115
void Posture_Analyzer::ads1115_set()
{  
    //bool status1 = ads.begin(address1,&I2C);
    //bool status2 = ads2.begin(address2);
    //bool status3 = ads3.begin(address3);
    //bool status4 = ads4.begin(address4);

    if (!ads1.begin(ADS1X15_ADDRESS1, &Wire))
    {
        ESP_LOGE(TAG, "Failed to initialize ADS1115_1.");
        while (1);
    }
    if (!ads2.begin(ADS1X15_ADDRESS2, &Wire))
    {
        ESP_LOGE(TAG, "Failed to initialize ADS1115_2.");
        while (1);
    }
    if (!ads3.begin(ADS1X15_ADDRESS3, &Wire))
    {
        ESP_LOGE(TAG, "Failed to initialize ADS1115_3.");
        while (1);
    }
    if (!ads4.begin(ADS1X15_ADDRESS4, &Wire))
    {
        ESP_LOGE(TAG, "Failed to initialize ADS1115_4.");
        while (1);
    }

    // The ADC input range (or gain) can be changed via the following
    // functions, but be careful never to exceed VDD +0.3V max, or to
    // exceed the upper and lower limits if you adjust the input range!
    // Setting these values incorrectly may destroy your ADC!
    //
    //                                          ADS1015          ADS1115
    //                                          -------          -------
    // GAIN_TWOTHIRDS  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // GAIN_ONE        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // GAIN_TWO        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // GAIN_FOUR       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // GAIN_EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // GAIN_SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    ads1.setGain(GAIN_ONE); 
    ads2.setGain(GAIN_ONE); 
    ads3.setGain(GAIN_ONE);
    ads4.setGain(GAIN_ONE);
    
    // RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
    // RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
    // RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
    // RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
    // RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
    // RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
    // RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
    // RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second
    ads1.setDataRate(RATE_ADS1115_860SPS);
    ads2.setDataRate(RATE_ADS1115_860SPS);
    ads3.setDataRate(RATE_ADS1115_860SPS);
    ads4.setDataRate(RATE_ADS1115_860SPS);
    
    // ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
    // ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000) ///< Differential P = AIN0, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000) ///< Differential P = AIN1, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000) ///< Differential P = AIN2, N = AIN3
    // ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
    // ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
    // ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
    // ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3
    // ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1);
}
void Posture_Analyzer::ads1115()
{
    for(int i=0;i<4;i++)
    {
        adc[i] = ads1.readADC_SingleEnded(i%4);
        volts[i] = ads1.computeVolts(adc[i]) * get_gain();
        data += String(volts[i]) + ",";
    }
    for(int i=4;i<8;i++)
    {
        adc[i] = ads2.readADC_SingleEnded(i%4);
        volts[i] = ads2.computeVolts(adc[i]) * get_gain();
        data += String(volts[i]) + ",";
    }
    for(int i=8;i<12;i++)
    {
        adc[i] = ads3.readADC_SingleEnded(i%4);
        volts[i] = ads3.computeVolts(adc[i]) * get_gain();
        data += String(volts[i]) + ",";
    }
    for(int i=12;i<16;i++)
    {
        adc[i] = ads4.readADC_SingleEnded(i%4);
        volts[i] = ads4.computeVolts(adc[i]) * get_gain();
        data += String(volts[i]) + ",";
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADXL345
void Posture_Analyzer::adxl345_set()
{
    if (!accel.begin(ADXL345_ADDRESS))
    {
        ESP_LOGE(TAG, "Failed to initialize ADXL345.");
        while (1);
    }

    /* Set the range to whatever is appropriate for your project */
    //  ADXL345_RANGE_16_G ///< +/- 16g
    //  ADXL345_RANGE_8_G  ///< +/- 8g
    //  ADXL345_RANGE_4_G  ///< +/- 4g
    //  ADXL345_RANGE_2_G  ///< +/- 2g (default value)
    accel.setRange(ADXL345_RANGE_2_G);
    
    //  ADXL345_DATARATE_3200_HZ  ///< 1600Hz Bandwidth   140�A IDD
    //  ADXL345_DATARATE_1600_HZ  ///<  800Hz Bandwidth    90�A IDD
    //  ADXL345_DATARATE_800_HZ  ///<  400Hz Bandwidth   140�A IDD
    //  ADXL345_DATARATE_400_HZ  ///<  200Hz Bandwidth   140�A IDD
    //  ADXL345_DATARATE_200_HZ  ///<  100Hz Bandwidth   140�A IDD
    //  ADXL345_DATARATE_100_HZ  ///<   50Hz Bandwidth   140�A IDD
    //  ADXL345_DATARATE_50_HZ   ///<   25Hz Bandwidth    90�A IDD
    //  ADXL345_DATARATE_25_HZ   ///< 12.5Hz Bandwidth    60�A IDD
    //  ADXL345_DATARATE_12_5_HZ ///< 6.25Hz Bandwidth    50�A IDD
    //  ADXL345_DATARATE_6_25HZ  ///< 3.13Hz Bandwidth    45�A IDD
    //  ADXL345_DATARATE_3_13_HZ ///< 1.56Hz Bandwidth    40�A IDD
    //  ADXL345_DATARATE_1_56_HZ ///< 0.78Hz Bandwidth    34�A IDD
    //  ADXL345_DATARATE_0_78_HZ ///< 0.39Hz Bandwidth    23�A IDD
    //  ADXL345_DATARATE_0_39_HZ ///< 0.20Hz Bandwidth    23�A IDD
    //  ADXL345_DATARATE_0_20_HZ ///< 0.10Hz Bandwidth    23�A IDD
    //  ADXL345_DATARATE_0_10_HZ ///< 0.05Hz Bandwidth    23�A IDD (default value)
    accel.setDataRate(ADXL345_DATARATE_100_HZ);
}
void Posture_Analyzer::adxl345()
{
    adxlmultiplier = ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    x = accel.getX() * adxlmultiplier;
    y = accel.getY() * adxlmultiplier;
    z = accel.getZ() * adxlmultiplier;
    data += String(x) + "," + String(y) + "," + String(z) + ",";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAX17048
void Posture_Analyzer::max17048_set()
{
    if (!maxlipo.begin(&Wire))
    {
        ESP_LOGE(TAG, "Failed to initialize MAX17048.");
        while (1);
    }
}
void Posture_Analyzer::max17048()
{
    voltage = maxlipo.cellVoltage();
    percentage = maxlipo.cellPercent();
    data += String(voltage) + "," + String(percentage) + ",";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Internal Temp
void Posture_Analyzer::internal_temp()
{
    uint8_t raw = temprature_sens_read();
    // ESP_LOGV(TAG, "Raw temperature value: %d", raw);
    temperature = (raw - 32) / 1.8f;
    // success = (raw != 128);

    // if (success && std::isfinite(temperature)) {
    //   this->publish_state(temperature);
    // } else {
    //   ESP_LOGD(TAG, "Ignoring invalid temperature (success=%d, value=%.1f)", success, temperature);
    //   if (!this->has_state()) {
    //     this->publish_state(NAN);
    //   }
    // }
    data += String(temperature);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Sensor
void Posture_Analyzer::sensor()
{
    if (this->sample_ != nullptr) this->sample_->publish_state(sayac);
    if (this->sample_sec_ != nullptr) this->sample_sec_->publish_state(sayac*1000/millis());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Posture_Analyzer::setup()
{
    uuid_set();
    bt_set();
    i2c_set();
    ads1115_set();
    adxl345_set();
    max17048_set();
}    
void Posture_Analyzer::dump_config()
{
    ESP_LOGI(TAG, "UUID: %s", uuid.toCharArray());
    ESP_LOGI(TAG, "Bluetooth Device name ready to pair: %s", get_bluetooth_name());
}
void Posture_Analyzer::loop()
{
}
void Posture_Analyzer::update()
{
    ads1115();
    adxl345();
    max17048();
    internal_temp();
    bt();
    sensor();
}

} //namespace posture_analyzer
} //namespace esphome