#include "esphome.h"
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
  
class ADXL345Sensor : public PollingComponent {
 public:
  Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
  Sensor *accel_x_sensor = new Sensor();
  Sensor *accel_y_sensor = new Sensor();
  Sensor *accel_z_sensor = new Sensor();

  ADXL345Sensor() : PollingComponent(1000) { }

  void setup() override {
    accel.begin();
    /* Set the range to whatever is appropriate for your project */
    accel.setRange(ADXL345_RANGE_16_G);  ///< +/- 16g
    //accel.setRange(ADXL345_RANGE_8_G);  ///< +/- 8g
    //accel.setRange(ADXL345_RANGE_4_G);  ///< +/- 4g
    //accel.setRange(ADXL345_RANGE_2_G);  ///< +/- 2g (default value)
    
    accel.setDataRate(ADXL345_DATARATE_3200_HZ);  ///< 1600Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_1600_HZ);  ///<  800Hz Bandwidth    90�A IDD
    //accel.setDataRate(ADXL345_DATARATE_800_HZ);  ///<  400Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_400_HZ);  ///<  200Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_200_HZ);  ///<  100Hz Bandwidth   140�A IDD
    //accel.setDataRate(ADXL345_DATARATE_100_HZ);  ///<   50Hz Bandwidth   140�A IDD
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
  }

  void update() override {
    sensors_event_t event;
    accel.getEvent(&event);

    accel_x_sensor->publish_state(event.acceleration.x);
    accel_y_sensor->publish_state(event.acceleration.y);
    accel_z_sensor->publish_state(event.acceleration.z);
  }

};