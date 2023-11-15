#include "esphome.h"
//#include "TCA9548A.h"
#include "Wire.h"

#define MUX_Address 0x70 // TCA9548A Encoders address
#define sda 16
#define scl 32

// Wire.begin(sda,scl);

void tcaselect(uint8_t bus){
    if (bus > 7) return;
    Wire.beginTransmission(MUX_Address);  // TCA9548A address
    Wire.write(1 << bus);          // send byte to select bus
    Wire.endTransmission();
}


