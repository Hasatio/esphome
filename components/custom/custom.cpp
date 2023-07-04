#include "custom.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custom {

static const char *TAG = "custom";

void Custom::setup(){

}

void Custom::loop(){

}

void Custom::write_state(float state){

}

void Custom::dump_config() {
    ESP_LOGCONFIG(TAG, "custom float output");
}

} //namespace custom
} //namespace esphome