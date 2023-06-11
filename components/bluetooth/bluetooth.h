#include "esphome.h"
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

int BT;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth off--Run `make menuconfig` to enable it
#endif

class MyBluetoothComponent : public Component {
public:
float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

void setup() override {
  SerialBT.begin("ESP32");
  ESP_LOGD("data", "Bluetooth is ready to Pair");
}
void loop() override{
  if(SerialBT.available())
  {
    /*
    BT = SerialBT.read();
    ESP_LOGD("data", "data: %d",BT);
    */
  }
}
};