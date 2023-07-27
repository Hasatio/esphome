#include "custom.h"

namespace esphome {
namespace custom {

static const char *TAG = "custom";

double vr;
void Custom::setup(){

}

void Custom::loop(){

    ESP_LOGD(TAG, "loop %f",vr);
}

void Custom::set_variables(double var){
vr=var;
    ESP_LOGD(TAG, "var: %f", var);
}

} //namespace custom
} //namespace esphome
