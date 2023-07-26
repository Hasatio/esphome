#include "custom.h"

namespace esphome {
namespace custom {

static const char *TAG = "custom";

double *vr;
void Custom::setup(){

    pinMode(13, OUTPUT);
    
    ESP_LOGD(TAG, "setup");
}

void Custom::loop(){

    ESP_LOGD(TAG, "loop %f",vr);
}

void Custom::write_state(float state){

    ESP_LOGD(TAG, "Empty custom float output %f",state);
    int value = state * 1024;
    analogWrite(13, value);
    this->set_level(state != this->inverted_ ? 1.0f : 0.0f);
}

void Custom::dump_config() {

}

void Custom::set_variables(double var){
vr=var;
    ESP_LOGD(TAG, "var: %f", var);
}

} //namespace custom
} //namespace esphome
