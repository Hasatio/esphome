#include "custom.h"

namespace esphome {
namespace custom {

static const char *TAG = "custom";

void Custom::setup(){

    pinMode(13, OUTPUT);
}

void Custom::loop(){

}

void Custom::write_state(float state){

    ESP_LOGD(TAG, "Empty custom float output",state);
    int value = state * 1024;
    analogWrite(13, value);
    this->set_level(state != this->inverted_ ? 1.0f : 0.0f);
}

void Custom::dump_config() {

}

void Custom::check_uart_settings(uint32_t baud_rate){

    ESP_LOGE(TAG, "  Invalid baud_rate: you have %u!", baud_rate);
  }

} //namespace custom
} //namespace esphome
