#include "veml7700.h"
#include "tca9548.h"

namespace esphome {
namespace veml7700 {

  Adafruit_VEML7700 veml = Adafruit_VEML7700();

void VEML7700::setup()
{
  // Wire.begin();
/*
  if (veml.begin()) {
  VEML7700Present = true;
  } else {
    return;
  }
*/
  
    this->set_i2c_address(VEML7700_ADDRESS);
    if (this->is_failed())
        return;

  // tcaselect(4);
  //Wire.beginTransmission(0x10);
  if (!veml.begin(this))
      ESP_LOGE(TAG,"Failed to initialize VEML7700.");
  else
      ESP_LOGI(TAG,"Successfulled to initialize VEML7700.");

  veml.setGain(VEML_GAIN); // in example is VEML7700_GAIN_1
  veml.setIntegrationTime(VEML_IT); // in example is VEML7700_IT_800MS
  //veml.readLux(); // fetch the first entry, but discard it - it will be wrong
}
void VEML7700::dump_config() {} 
void VEML7700::loop() {} 
void VEML7700::update() 
{
//  if (VEML7700Present) {

    this->set_i2c_address(VEML7700_ADDRESS);
    if (this->is_failed())
        return;

  // tcaselect(4);
  if (this->Lux_ != nullptr)    { this->Lux_->publish_state(veml.readLux()); }
  if (this->White_ != nullptr)  { this->White_->publish_state(veml.readWhite()); }
  if (this->Als_ != nullptr)    { this->Als_->publish_state(veml.readALS()); }

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

}  // namespace veml7700
}  // namespace esphome