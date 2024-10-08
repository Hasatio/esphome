#include "water_quality.h"
#include "wq_i2c.h"

namespace esphome {
namespace water_quality {

    // Analog ana;
    // Digital digi;

// WQ_I2C::WQ_I2C(WaterQuality *parent) : i2c::I2CDevice(), parent_(parent) {}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADS1115

void WaterQuality::ADS1115_Setup(uint8_t address)
{
    this->set_i2c_address(address);
    if (this->is_failed())
        return;

    ESP_LOGCONFIG(TAG, "Setting up ADS1115...");

    uint16_t value;

    if (!this->read_byte_16(ADS1115_REGISTER_CONVERSION, &value))
    {
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
    config |= this->get_gain() << 9;

    if (this->get_continuous_mode()) 
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
    config |= this->get_data_rate() << 5;

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

    if (!this->write_byte_16(ADS1115_REGISTER_CONFIG, config))
    {
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
float WaterQuality::ADS1115_Read()
{
    uint16_t config = this->prev_config_;
    // uint16_t config = 0b0000000011100011;
    // Multiplexer
    //        0bxBBBxxxxxxxxxxxx
    config &= 0b1000111111111111;
    config |= this->get_multiplexer() << 12;

    // Gain
    //        0bxxxxBBBxxxxxxxxx
    config &= 0b1111000111111111;
    config |= this->get_gain() << 9;

    if (!this->get_continuous_mode()) {
        // Start conversion
        config |= 0b1000000000000000;
    }

    if (!this->get_continuous_mode() || this->prev_config_ != config)
    {
        if (!this->write_byte_16(ADS1115_REGISTER_CONFIG, config))
        {
            this->status_set_warning();
            return NAN;
        }

        this->prev_config_ = config;

        // about 1.2 ms with 860 samples per second
        delay(2);
        
        // in continuous mode, conversion will always be running, rely on the delay
        // to ensure conversion is taking place with the correct settings
        // can we use the rdy pin to trigger when a conversion is done?
        if (!this->get_continuous_mode())
        {
            uint32_t start = millis();
            while (this->read_byte_16(ADS1115_REGISTER_CONFIG, &config) && (config >> 15) == 0)
            {
                if (millis() - start > 100)
                {
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

    if (this->get_resolution() == ADS1015_12_BITS)
    {
        bool negative = (raw_conversion >> 15) == 1;

        // shift raw_conversion as it's only 12-bits, left justified
        raw_conversion = raw_conversion >> (16 - ADS1015_12_BITS);

        // check if number was negative in order to keep the sign
        if (negative)
        {
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
    switch (this->gain_)
    {
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
    // ESP_LOGI(TAG, "volts: %f", millivolts / 1e3f);
    
}
void WaterQuality::ADS1115_Driver(float analog_voltage[])
{
    this->set_i2c_address(ADS1X15_ADDRESS1);
    if (this->is_failed())
        return;

    for (uint8_t i = 0; i < 4; i++)
    { 
        this->set_multiplexer(static_cast<ADS1115_Multiplexer>(ADS1115_MULTIPLEXER_P0_NG + i));

        float v = ADS1115_Read();
        if (!std::isnan(v)) 
        {
            analog_voltage[i] = v;
            // ESP_LOGD(TAG, "Voltage%d: %f", i, v);
            // this->publish_state(v);
        }
    }

    this->set_i2c_address(ADS1X15_ADDRESS2);
    if (this->is_failed())
        return;

    for (uint8_t i = 0; i < 4; i++)
    { 
        this->set_multiplexer(static_cast<ADS1115_Multiplexer>(ADS1115_MULTIPLEXER_P0_NG + i));

        float v = ADS1115_Read();
        if (!std::isnan(v)) 
        {
            analog_voltage[i + 4] = v;
            // ESP_LOGD(TAG, "Voltage%d: %f", i + 4, v);
            // this->publish_state(v);
        }
    }
    // ESP_LOGD(TAG, "volt = %f", analog_voltage[1]);
    // ana.Analog_Input_Driver(analog_voltage);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

void WaterQuality::MCP23008_Setup(uint8_t address)
{
    this->set_i2c_address(address);
    if (this->is_failed())
        return;

    ESP_LOGCONFIG(TAG, "Setting up MCP23008...");

    uint8_t iocon;
    if (!this->read_byte(MCP23008_IOCON, &iocon))
    {
        this->mark_failed();
        return;
    }

    uint8_t reg_value = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        reg_value |= 1 << i;
    }
    for (uint8_t i = 4; i < 8; i++)
    {
        reg_value &= ~(1 << i);
    }

    this->write_byte(MCP23008_GPIO, reg_value);
    this->write_byte(MCP23008_IODIR, reg_value);
    this->write_byte(MCP23008_GPPU, reg_value);
    this->write_byte(MCP23008_OLAT, reg_value);

    // Read current output register state
    this->read_byte(MCP23008_OLAT, &this->olat_);


    this->write_byte(MCP23008_IPOL, 0x00);
    this->write_byte(MCP23008_GPINTEN, 0x00);
    this->write_byte(MCP23008_DEFVAL, 0x00);
    this->write_byte(MCP23008_INTCON, 0x00);

    if (this->open_drain_ints_)
        // enable open-drain interrupt pins, 3.3V-safe
        this->write_byte(MCP23008_IOCON, 0x04);

    this->write_byte(MCP23008_INTF, 0x00);
    this->write_byte(MCP23008_INTCAP, 0x00);
}
void WaterQuality::MCP23008_Read(bool value[])
{
    uint8_t value_;

    this->read_byte(MCP23008_GPIO, &value_);

    for (uint8_t i = 0; i < 4; i++)
    {
        value[i] = value_ & (1 << i);
        // ESP_LOGD(TAG, "in[%d] = %d", i, digital[i] ? 1 : 0);
    }
}
void WaterQuality::MCP23008_Write(bool value[])
{
    uint8_t reg_value = this->olat_;

    for (uint8_t i = 0; i < 4; i++)
    {
        // uint8_t olat_;
        // this->read_byte(MCP23008_OLAT, &this->olat_);

        if (value[i])
            reg_value |= 1 << (i + 4);
        else
            reg_value &= ~(1 << (i + 4));
    }

    if (reg_value != this->olat_)
    {
        // this->write_byte(MCP23008_GPIO, reg_value);
        this->write_byte(MCP23008_OLAT, reg_value);
        this->olat_ = reg_value;
    }
}
void WaterQuality::MCP23008_pin_interrupt_mode(uint8_t pin, MCP23008_InterruptMode interrupt_mode)
{
    uint8_t gpinten = MCP23008_GPINTEN;
    uint8_t intcon = MCP23008_INTCON;
    uint8_t defval = MCP23008_DEFVAL;

    // switch (interrupt_mode)
    // {
    //   case MCP23008_CHANGE:
    //     this->MCP23008_update_reg(pin, true, gpinten);
    //     this->MCP23008_update_reg(pin, false, intcon);
    //     break;
    //   case MCP23008_RISING:
    //     this->MCP23008_update_reg(pin, true, gpinten);
    //     this->MCP23008_update_reg(pin, true, intcon);
    //     this->MCP23008_update_reg(pin, true, defval);
    //     break;
    //   case MCP23008_FALLING:
    //     this->MCP23008_update_reg(pin, true, gpinten);
    //     this->MCP23008_update_reg(pin, true, intcon);
    //     this->MCP23008_update_reg(pin, false, defval);
    //     break;
    //   case MCP23008_NO_INTERRUPT:
    //     this->MCP23008_update_reg(pin, false, gpinten);
    //     break;
    // }
}
void WaterQuality::MCP23008_Driver(bool digital[])
{
    this->set_i2c_address(MCP23008_ADDRESS);
    if (this->is_failed())
        return;
    
    MCP23008_Write(digital);
    
    bool value[4];
    MCP23008_Read(value);

    for (uint8_t i = 0; i < 4; i++)
    {
        digital[i] = value[i];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PCA9685

void WaterQuality::PCA9685_Setup(uint8_t address)
{
    this->set_i2c_address(address);
    if (this->is_failed())
        return;

    ESP_LOGCONFIG(TAG, "Setting up PCA9685OutputComponent...");

    ESP_LOGV(TAG, "  Resetting devices...");
    if (!this->write_bytes(PCA9685_REGISTER_SOFTWARE_RESET, nullptr, 0)) {
        this->mark_failed();
        return;
    }

    if (!this->write_byte(PCA9685_REGISTER_MODE1, PCA9685_MODE1_RESTART)) {
        this->mark_failed();
        return;
    }

    if (!this->write_byte(PCA9685_REGISTER_MODE1, PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC)) {
        this->mark_failed();
        return;
    }
    uint8_t mode_ = PCA9685_MODE_OUTPUT_ONACK | PCA9685_MODE_OUTPUT_TOTEM_POLE;
    if (!this->write_byte(PCA9685_REGISTER_MODE2, mode_)) {
        this->mark_failed();
        return;
    }

    uint8_t mode1;
    if (!this->read_byte(PCA9685_REGISTER_MODE1, &mode1)) {
        this->mark_failed();
        return;
    }
    mode1 = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    if (!this->write_byte(PCA9685_REGISTER_MODE1, mode1)) {
        this->mark_failed();
        return;
    }

    int pre_scaler = 3;
    if (this->extclk_) {
        mode1 = mode1 | PCA9685_MODE1_EXTCLK;
        if (!this->write_byte(PCA9685_REGISTER_MODE1, mode1)) {
        this->mark_failed();
        return;
        }
    } else {
        pre_scaler = static_cast<int>((25000000 / (4096 * this->frequency_)) - 1);
        pre_scaler = clamp(pre_scaler, 3, 255);

        ESP_LOGV(TAG, "Prescaler: %d", pre_scaler);
    }
    if (!this->write_byte(PCA9685_REGISTER_PRE_SCALE, pre_scaler)) {
        this->mark_failed();
        return;
    }

    mode1 = (mode1 & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART;
    if (!this->write_byte(PCA9685_REGISTER_MODE1, mode1)) {
        this->mark_failed();
        return;
    }
    delayMicroseconds(500);
}
void WaterQuality::PCA9685_Write()
{
    if (!this->update_)
        return;

    const uint8_t min_channel{0};
    const uint8_t max_channel{16};
    const uint16_t max_duty = 4096;
    for (uint8_t i = min_channel; i <= max_channel; i++)
    {
        uint16_t phase_begin = uint16_t(i - min_channel) / max_channel * max_duty;
        uint16_t phase_end;
        uint16_t amount = this->pwm_amounts_[i];
        if (amount == 0)
        {
            phase_end = max_duty;
        }
        else if (amount >= max_duty)
        {
            phase_begin = max_duty;
            phase_end = 0;
        }
        else
        {
            phase_end = phase_begin + amount;
            if (phase_end >= max_duty)
                phase_end -= max_duty;
        }

        // ESP_LOGI(TAG, "Channel %02u: amount=%04u phase_begin=%04u phase_end=%04u", i, amount, phase_begin, phase_end);

        uint8_t data[4];
        data[0] = phase_begin & 0xFF;
        data[1] = (phase_begin >> 8) & 0xFF;
        data[2] = phase_end & 0xFF;
        data[3] = (phase_end >> 8) & 0xFF;

        uint8_t reg = PCA9685_REGISTER_LED0 + 4 * i;
        if (!this->write_bytes(reg, data, 4))
        {
            this->status_set_warning();
            return;
        }
    }

    this->status_clear_warning();
    this->update_ = false;
}
void WaterQuality::PCA9685_Driver(float state[])
{
    this->set_i2c_address(PCA9685_I2C_ADDRESS);
    if (this->is_failed())
        return;

    const uint16_t max_duty = 4096;
    for (uint8_t i = 0; i < 16; i++)
    {
        uint16_t duty = static_cast<uint16_t>(roundf(state[i] * max_duty));
        if (this->pwm_amounts_[i] != duty)
            this->update_ = true;
            
        this->pwm_amounts_[i] = duty;
        PCA9685_Write();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace water_quality
}  // namespace esphome