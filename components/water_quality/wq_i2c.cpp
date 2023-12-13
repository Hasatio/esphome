#include "water_quality.h"
#include "wq_i2c.h"
#include "wq_analog.h"
#include "wq_digital.h"

namespace esphome {
namespace water_quality {

    // Analog ana;
    // Digital digi;
    WaterQuality wq;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

    // Adafruit_MCP23008 mcp;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void EC10() {/*DFRobot_EC10 ec;*/}
// void EC() {DFRobot_EC ec;}
    
    // DFRobot_EC ec;
    // DFRobot_PH ph;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548
void tcaselect(uint8_t bus)
{
    if (bus > 7) return;
    Wire.beginTransmission(TCA9548_ADDRESS);
    Wire.write(1 << bus);
    Wire.endTransmission();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WaterQuality::ADS1115_Setup(uint8_t address)
{
  this->set_i2c_address(address);

  ESP_LOGCONFIG(TAG, "Setting up ADS1115...");
  uint16_t value;
  if (!this->read_byte_16(ADS1115_REGISTER_CONVERSION, &value)) {
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "Configuring ADS1115...");
  set_gain(ADS1115_GAIN_6P144);
  set_continuous_mode(true);
  set_data_rate(ADS1115_DATA_RATE_860_SPS);
  set_resolution(ADS1115_16_BITS);

  uint16_t config = 0;
  // Clear single-shot bit
  //        0b0xxxxxxxxxxxxxxx
  config |= 0b0000000000000000;
  // Setup multiplexer
  //        0bx000xxxxxxxxxxxx
  config |= ADS1115_MULTIPLEXER_P0_N1 << 12;

  // Setup Gain
  //        0bxxxx000xxxxxxxxx
  config |= ADS1115_GAIN_6P144 << 9;

  if (this->continuous_mode_) 
  {
    // Set continuous mode
    //        0bxxxxxxx0xxxxxxxx
    config |= 0b0000000000000000;
  } 
  else 
  {
    // Set singleshot mode
    //        0bxxxxxxx1xxxxxxxx
    config |= 0b0000000100000000;
  }

  // Set data rate - 860 samples per second (we're in singleshot mode)
  //        0bxxxxxxxx100xxxxx
  config |= ADS1115_DATA_RATE_860_SPS << 5;

  // Set comparator mode - hysteresis
  //        0bxxxxxxxxxxx0xxxx
  config |= 0b0000000000000000;

  // Set comparator polarity - active low
  //        0bxxxxxxxxxxxx0xxx
  config |= 0b0000000000000000;

  // Set comparator latch enabled - false
  //        0bxxxxxxxxxxxxx0xx
  config |= 0b0000000000000000;

  // Set comparator que mode - disabled
  //        0bxxxxxxxxxxxxxx11
  config |= 0b0000000000000011;

  if (!this->write_byte_16(ADS1115_REGISTER_CONFIG, config)) {
    this->mark_failed();
    return;
  }
  this->prev_config_ = config;
    
//     //                                          ADS1015          ADS1115
//     //                                          -------          -------
//     // GAIN_TWOTHIRDS  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
//     // GAIN_ONE        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
//     // GAIN_TWO        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
//     // GAIN_FOUR       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
//     // GAIN_EIGHT      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
//     // GAIN_SIXTEEN    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    
//     // RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
//     // RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
//     // RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
//     // RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
//     // RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
//     // RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
//     // RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
//     // RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second
    
//     // ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
//     // ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000) ///< Differential P = AIN0, N = AIN3
//     // ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000) ///< Differential P = AIN1, N = AIN3
//     // ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000) ///< Differential P = AIN2, N = AIN3
//     // ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
//     // ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
//     // ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
//     // ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3

//     // AnInEC_Type == 1? EC():EC10();
//     // // AnInEC_Type == 10? EC10();

//     // ec.begin();
//     // ph.begin();
}
float WaterQuality::ADS1115_Read(ADS1115_Multiplexer multi)
{
  uint16_t config = this->prev_config_;
  // uint16_t config = 0b0000000011100011;
  // Multiplexer
  //        0bxBBBxxxxxxxxxxxx
  config &= 0b1000111111111111;
  config |= multi << 12;

  // Gain
  //        0bxxxxBBBxxxxxxxxx
  config &= 0b1111000111111111;
  config |= (ADS1115_GAIN_6P144) << 9;

  if (!this->continuous_mode_) {
    // Start conversion
    config |= 0b1000000000000000;
  }

  if (!this->continuous_mode_ || this->prev_config_ != config) {
    if (!this->write_byte_16(ADS1115_REGISTER_CONFIG, config)) {
      this->status_set_warning();
      return NAN;
    }
    this->prev_config_ = config;

    // about 1.2 ms with 860 samples per second
    delay(2);
    
    // in continuous mode, conversion will always be running, rely on the delay
    // to ensure conversion is taking place with the correct settings
    // can we use the rdy pin to trigger when a conversion is done?
    if (!this->continuous_mode_) {
      uint32_t start = millis();
      while (this->read_byte_16(ADS1115_REGISTER_CONFIG, &config) && (config >> 15) == 0) {
        if (millis() - start > 100) {
          ESP_LOGW(TAG, "Reading ADS1115 timed out");
          this->status_set_warning();
          return NAN;
        }
        yield();
      }
    }
  }

  uint16_t raw_conversion;
  if (!this->read_byte_16(ADS1115_REGISTER_CONVERSION, &raw_conversion)) {
    this->status_set_warning();
    return NAN;
  }

  if (this->get_resolution() == ADS1015_12_BITS) {
    bool negative = (raw_conversion >> 15) == 1;

    // shift raw_conversion as it's only 12-bits, left justified
    raw_conversion = raw_conversion >> (16 - ADS1015_12_BITS);

    // check if number was negative in order to keep the sign
    if (negative) {
      // the number was negative
      // 1) set the negative bit back
      raw_conversion |= 0x8000;
      // 2) reset the former (shifted) negative bit
      raw_conversion &= 0xF7FF;
    }
  }

  auto signed_conversion = static_cast<int16_t>(raw_conversion);

  float millivolts;
  float divider = (this->get_resolution() == ADS1115_16_BITS) ? 32768.0f : 2048.0f;
  switch (this->get_gain()) {
    case ADS1115_GAIN_6P144:
      millivolts = (signed_conversion * 6144) / divider;
      break;
    case ADS1115_GAIN_4P096:
      millivolts = (signed_conversion * 4096) / divider;
      break;
    case ADS1115_GAIN_2P048:
      millivolts = (signed_conversion * 2048) / divider;
      break;
    case ADS1115_GAIN_1P024:
      millivolts = (signed_conversion * 1024) / divider;
      break;
    case ADS1115_GAIN_0P512:
      millivolts = (signed_conversion * 512) / divider;
      break;
    case ADS1115_GAIN_0P256:
      millivolts = (signed_conversion * 256) / divider;
      break;
    default:
      millivolts = NAN;
  }

  this->status_clear_warning();
  // ESP_LOGI(TAG, "config: %x", config);
  return millivolts / 1e3f;
}
void WaterQuality::ADS1115_Driver(float analog_voltage[])
{
  this->set_i2c_address(ADS1X15_ADDRESS1);
  for (size_t i = 0; i < 4; i++)
  {
      float v = ADS1115_Read(static_cast<ADS1115_Multiplexer>(i + 4));
      if (!std::isnan(v)) 
      {
          analog_voltage[i] = v;
          // ESP_LOGD(TAG, "Voltage%d: %f", i, v);
          // this->publish_state(v);
      }
  }
  this->set_i2c_address(ADS1X15_ADDRESS2);
  for (size_t i = 0; i < 4; i++)
  {
      float v = ADS1115_Read(static_cast<ADS1115_Multiplexer>(i + 4));
      if (!std::isnan(v)) 
      {
          analog_voltage[i + 4] = v;
          // ESP_LOGD(TAG, "Voltage%d: %f", i + 4, v);
          // this->publish_state(v);
      }
  }
  // ESP_LOGD(TAG,"volt = %f", analog_voltage[1]);
  // ana.Analog_Input_Driver(analog_voltage);
}

void Data::test()
{
  WaterQuality *wq;
    // LOG_I2C_DEVICE(wq->get_address());
    // if (wq->is_failed())
    //     ESP_LOGE(TAG, "var");
    // else
    //     ESP_LOGI(TAG, "yok");
}

void WaterQuality::MCP23008_Setup()
{
  this->set_i2c_address(MCP23008_ADDRESS);

  ESP_LOGCONFIG(TAG, "Setting up MCP23008...");
  uint8_t iocon;
  if (!this->MCP23008_read_reg(MCP23008_IOCON, &iocon)) {
    this->mark_failed();
    return;
  }

  // Read current output register state
  this->MCP23008_read_reg(MCP23008_OLAT, &this->olat_);

  if (this->open_drain_ints_) {
    // enable open-drain interrupt pins, 3.3V-safe
    this->MCP23008_write_reg(MCP23008_IOCON, 0x04);
  }

  for (size_t i = 0; i < 4; i++)
  {
    this->MCP23008_update_reg(i, true, MCP23008_IODIR);
    this->MCP23008_update_reg(i, true, MCP23008_GPPU);
    this->MCP23008_update_reg(i, true, MCP23008_OLAT);
  }
  for (size_t i = 4; i < 8; i++)
  {
    this->MCP23008_update_reg(i, false, MCP23008_IODIR);
    this->MCP23008_update_reg(i, false, MCP23008_OLAT);
  }
}
bool WaterQuality::MCP23008_read_reg(uint8_t reg, uint8_t *value)
{
  this->set_i2c_address(MCP23008_ADDRESS);
  if (this->is_failed())
    return false;

  return this->read_byte(reg, value);
}
bool WaterQuality::MCP23008_write_reg(uint8_t reg, uint8_t value)
{
  this->set_i2c_address(MCP23008_ADDRESS);
  if (this->is_failed())
    return false;

  return this->write_byte(reg, value);
}


bool WaterQuality::MCP23008_digital_read(uint8_t pin) {
  uint8_t bit = pin % 8;
  uint8_t value = 0;
  this->MCP23008_read_reg(MCP23008_GPIO, &value);
  return value & (1 << bit);
}

void WaterQuality::MCP23008_digital_write(uint8_t pin, bool value) 
{
  this->MCP23008_update_reg(pin, value, MCP23008_OLAT);
}

void WaterQuality::MCP23008_pin_interrupt_mode(uint8_t pin, MCP23008_InterruptMode interrupt_mode) {
  uint8_t gpinten = MCP23008_GPINTEN;
  uint8_t intcon = MCP23008_INTCON;
  uint8_t defval = MCP23008_DEFVAL;

  switch (interrupt_mode)
  {
    case MCP23008_CHANGE:
      this->MCP23008_update_reg(pin, true, gpinten);
      this->MCP23008_update_reg(pin, false, intcon);
      break;
    case MCP23008_RISING:
      this->MCP23008_update_reg(pin, true, gpinten);
      this->MCP23008_update_reg(pin, true, intcon);
      this->MCP23008_update_reg(pin, true, defval);
      break;
    case MCP23008_FALLING:
      this->MCP23008_update_reg(pin, true, gpinten);
      this->MCP23008_update_reg(pin, true, intcon);
      this->MCP23008_update_reg(pin, false, defval);
      break;
    case MCP23008_NO_INTERRUPT:
      this->MCP23008_update_reg(pin, false, gpinten);
      break;
  }
}

void WaterQuality::MCP23008_update_reg(uint8_t pin, bool pin_value, uint8_t reg_addr) {
  uint8_t bit = pin % 8;
  uint8_t reg_value = 0;
  if (reg_addr == MCP23008_OLAT)
  {
    reg_value = this->olat_;
  } 
  else
    this->MCP23008_read_reg(reg_addr, &reg_value);

  if (pin_value)
    reg_value |= 1 << bit;
  else
    reg_value &= ~(1 << bit);

  this->MCP23008_write_reg(reg_addr, reg_value);

  if (reg_addr == MCP23008_OLAT)
    this->olat_ = reg_value;
}


void WaterQuality::MCP23008_Driver()
{
  bool* digital = dig.get_Digital_Out();
  for (size_t i = 4; i < 8; i++)
  {
    if(digital[i - 4])
      this->MCP23008_update_reg(i, true, MCP23008_GPIO);
    else
      this->MCP23008_update_reg(i, false, MCP23008_GPIO);

    bool value[8]; 
    this->MCP23008_read_reg(MCP23008_GPIO, &value[i]);
      ESP_LOGD(TAG,"value %d = %d", i, value[i]);
    value[i + 4] = MCP23008_digital_read(i - 4);
      ESP_LOGD(TAG,"value %d = %d", i, value[i + 4]);
  }

    // mcp.digitalWrite(4,LOW);
    // mcp.digitalWrite(5,LOW);
    // mcp.digitalWrite(6,LOW);
    // mcp.digitalWrite(7,LOW);

    // for(size_t i = 0; i < 4; i++)
    // {
    //     digi.DigIn_Read[i] = mcp.digitalRead(i);
    //     // ESP_LOGD(TAG,"dig input %d = %d", i, DigIn_Read[i]);
    // }

    // for(size_t i = 0; i < 4; i++)
    // {
    //     if (digi.DigOut_Status[i] == 1)
    //     {
    //         mcp.digitalWrite(i + 4, HIGH);
    //     }
    //     else
    //     { 
    //         mcp.digitalWrite(i + 4, LOW);
    //     }
    //     // ESP_LOGD(TAG,"dig output %d = %d", i, DigOut_Status[i]);
    // }
}

}  // namespace water_quality
}  // namespace esphome