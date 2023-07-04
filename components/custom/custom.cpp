#include "custom.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custom {

static const char *TAG = "custom.output";

void Custom::setup(){

}

void Custom::write_state(float state){

}

void Custom::dump_config() {
    ESP_LOGCONFIG(TAG, "Empty custom float output");
}

} //namespace empty_float_output
} //namespace esphome
