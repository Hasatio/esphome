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

void Custom::set_variables(double a){
    vr=a;
    ESP_LOGD(TAG, "var: %f", a);
}

} //namespace custom
} //namespace esphome
