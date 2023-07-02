#include "esphome.h"
#include "mux.h"
#include <Adafruit_VEML7700.h>

#define VEML_POLL_RATE 1000 // 27000 makes a good rate, def 15000-17000
#define VEML_GAIN VEML7700_GAIN_1 // GAIN_1 is 'default'
#define VEML_IT VEML7700_IT_100MS // 100MS is 'default'
// gain 1, IT 100mS, means the max light level reported will be 3775

class VEML7700Sensor : public PollingComponent, public Sensor {
 protected:
  bool VEML7700Present = false, VEML7700FirstRead = true;

 public:
  Adafruit_VEML7700 veml = Adafruit_VEML7700();  // constructor

  VEML7700Sensor() : PollingComponent(VEML_POLL_RATE) {}
  Sensor *veml7700_lux = new Sensor();
  Sensor *veml7700_white = new Sensor();
  Sensor *veml7700_raw_als = new Sensor();
  // Sensor *veml7700_gain = new Sensor();
  // Sensor *veml7700_it = new Sensor();
  // Sensor *veml7700_interrupt_status = new Sensor();

  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  void setup() override {
/*    if (veml.begin()) {
      VEML7700Present = true;
    } else {
      return;
    }*/
    
    tcaselect(4);
    //Wire.beginTransmission(0x10);
    veml.begin();
    veml.setGain(VEML_GAIN); // in example is VEML7700_GAIN_1
    veml.setIntegrationTime(VEML_IT); // in example is VEML7700_IT_800MS
    //veml.readLux(); // fetch the first entry, but discard it - it will be wrong
  }

  void update() override {    // This will be called every "update_interval" milliseconds.
//    if (VEML7700Present) {

    tcaselect(4);
    veml7700_lux->publish_state(veml.readLux());
    veml7700_white->publish_state(veml.readWhite());
    veml7700_raw_als->publish_state(veml.readALS());
    // veml7700_gain->publish_state(veml.getGain());
    // veml7700_it->publish_state(veml.getIntegrationTime());
    // veml7700_interrupt_status->publish_state(veml.interruptStatus());

    // uint16_t irq = veml.interruptStatus();
    // if (irq & VEML7700_INTERRUPT_LOW) {
    //   // Serial.println("** Low threshold"); 
    // }
    // if (irq & VEML7700_INTERRUPT_HIGH) {
    //   // Serial.println("** High threshold"); 
    // }
//    }
  }

  // void loop() override {   // This will be called by App.loop(), very rapidly
  // }
};
