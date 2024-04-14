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

    for (size_t i = 0; i < 4; i++)
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

    for (size_t i = 0; i < 4; i++)
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
    for (size_t i = 0; i < 4; i++)
    {
        reg_value |= 1 << i;
    }
    for (size_t i = 4; i < 8; i++)
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
uint8_t WaterQuality::MCP23008_Read()
{
    uint8_t value;
    this->read_byte(MCP23008_GPIO, &value);

    return value;
}
void WaterQuality::MCP23008_Write(bool value[])
{
    uint8_t reg_value = this->olat_;

    for (size_t i = 0; i < 4; i++)
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
    
    uint8_t value = MCP23008_Read();
    for (size_t i = 0; i < 4; i++)
    {
        digital[i] = value & (1 << i);
        // ESP_LOGD(TAG, "in[%d] = %d", i, digital[i] ? 1 : 0);
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
    for (size_t i = min_channel; i <= max_channel; i++)
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
//  EZOPMP
void WaterQuality::find() { this->queue_command_(EZO_PMP_COMMAND_FIND, 0, 0, true); }
void WaterQuality::dose_continuously()
{
    this->queue_command_(EZO_PMP_COMMAND_DOSE_CONTINUOUSLY, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}
void WaterQuality::dose_volume(float volume)
{
    this->queue_command_(EZO_PMP_COMMAND_DOSE_VOLUME, volume, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}
void WaterQuality::dose_volume_over_time(float volume, int duration)
{
    this->queue_command_(EZO_PMP_COMMAND_DOSE_VOLUME_OVER_TIME, volume, duration, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}
void WaterQuality::dose_with_constant_flow_rate(float volume, int duration)
{
    this->queue_command_(EZO_PMP_COMMAND_DOSE_WITH_CONSTANT_FLOW_RATE, volume, duration, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}
void WaterQuality::set_calibration_volume(float volume)
{
    this->queue_command_(EZO_PMP_COMMAND_SET_CALIBRATION_VOLUME, volume, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_CALIBRATION_STATUS, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_MAX_FLOW_RATE, 0, 0, true);
}
void WaterQuality::clear_total_volume_dosed()
{
    this->queue_command_(EZO_PMP_COMMAND_CLEAR_TOTAL_VOLUME_DOSED, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_TOTAL_VOLUME_DOSED, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_ABSOLUTE_TOTAL_VOLUME_DOSED, 0, 0, true);
}
void WaterQuality::clear_calibration()
{
    this->queue_command_(EZO_PMP_COMMAND_CLEAR_CALIBRATION, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_CALIBRATION_STATUS, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_MAX_FLOW_RATE, 0, 0, true);
}
void WaterQuality::pause_dosing()
{
    this->queue_command_(EZO_PMP_COMMAND_PAUSE_DOSING, 0, 0, true);
    this->queue_command_(EZO_PMP_COMMAND_READ_PAUSE_STATUS, 0, 0, true);
}
void WaterQuality::stop_dosing() { this->queue_command_(EZO_PMP_COMMAND_STOP_DOSING, 0, 0, true); }
void WaterQuality::change_i2c_address(int address)
{
    this->queue_command_(EZO_PMP_COMMAND_CHANGE_I2C_ADDRESS, 0, address, true);
}

void WaterQuality::clear_current_command_()
{
    this->current_command_ = EZO_PMP_COMMAND_NONE;
    this->is_waiting_ = false;
}
void WaterQuality::read_command_result_()
{
    this->set_i2c_address(EZOPMP_I2C_ADDRESS);
    if (this->is_failed())
        return;

    // ESP_LOGI(TAG, "total_volume_dosed_ = %f", this->total_volume_dosed_);
    // ESP_LOGI(TAG, "absolute_total_volume_dosed_ = %f", this->absolute_total_volume_dosed_);

    uint8_t response_buffer[21] = {'\0'};

    response_buffer[0] = 0;
    if (!this->read_bytes_raw(response_buffer, 20))
    {
        // ESP_LOGE(TAG, "read error");
        
        for (size_t i = 0; i < 21; i++)
            if (this->command2_[i] != response_buffer[i] /*&& response_buffer[0] <= 1*/)
            {
                ESP_LOGE(TAG, "response_buffer = %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", response_buffer[0], response_buffer[1], response_buffer[2], response_buffer[3], response_buffer[4], response_buffer[5], response_buffer[6], response_buffer[7], response_buffer[8], response_buffer[9], response_buffer[10], response_buffer[11], response_buffer[12], response_buffer[13], response_buffer[14], response_buffer[15], response_buffer[16], response_buffer[17], response_buffer[18], response_buffer[19], response_buffer[20]);
                ESP_LOGI(TAG, "Read Response from device: %s", (char *) response_buffer);
            
                this->command2_[i] = response_buffer[i];
            }

        this->clear_current_command_();
        return;
    }

    for (size_t i = 0; i < 21; i++)
        if (this->command2_[i] != response_buffer[i] /*&& response_buffer[0] <= 1*/)
        {
            ESP_LOGE(TAG, "response_buffer = %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", response_buffer[0], response_buffer[1], response_buffer[2], response_buffer[3], response_buffer[4], response_buffer[5], response_buffer[6], response_buffer[7], response_buffer[8], response_buffer[9], response_buffer[10], response_buffer[11], response_buffer[12], response_buffer[13], response_buffer[14], response_buffer[15], response_buffer[16], response_buffer[17], response_buffer[18], response_buffer[19], response_buffer[20]);
            ESP_LOGI(TAG, "Read Response from device: %s", (char *) response_buffer);
        
            this->command2_[i] = response_buffer[i];
        }

    switch (response_buffer[0])
    {
        case 254:
            return;  // keep waiting
        case 1:
            break;
        case 2:
            ESP_LOGE(TAG, "device returned a syntax error");
            this->clear_current_command_();
            return;
        case 255:
            // ESP_LOGE(TAG, "device returned no data");
            this->clear_current_command_();
            return;
        default:
            // ESP_LOGE(TAG, "device returned an unknown response: %d", response_buffer[0]);
            this->clear_current_command_();
            return;
    }

    char first_parameter_buffer[10] = {'\0'};
    char second_parameter_buffer[10] = {'\0'};
    char third_parameter_buffer[10] = {'\0'};

    first_parameter_buffer[0] = '\0';
    second_parameter_buffer[0] = '\0';
    third_parameter_buffer[0] = '\0';

    int current_parameter = 1;

    size_t position_in_parameter_buffer = 0;
    // some sensors return multiple comma-separated values, terminate string after first one
    for (size_t i = 1; i < sizeof(response_buffer) - 1; i++)
    {
        char current_char = response_buffer[i];

        if (current_char == '\0')
        {
            ESP_LOGI(TAG, "Read Response from device: %s", (char *) response_buffer);
            ESP_LOGI(TAG, "First Component: %s", (char *) first_parameter_buffer);
            ESP_LOGI(TAG, "Second Component: %s", (char *) second_parameter_buffer);
            ESP_LOGI(TAG, "Third Component: %s", (char *) third_parameter_buffer);

            break;
        }

        if (current_char == ',')
        {
            current_parameter++;
            position_in_parameter_buffer = 0;
            continue;
        }

        ESP_LOGI(TAG, "current_parameter: %d", current_parameter);

        switch (current_parameter)
        {
            case 1:
                first_parameter_buffer[position_in_parameter_buffer] = current_char;
                first_parameter_buffer[position_in_parameter_buffer + 1] = '\0';
                break;
            case 2:
                second_parameter_buffer[position_in_parameter_buffer] = current_char;
                second_parameter_buffer[position_in_parameter_buffer + 1] = '\0';
                break;
            case 3:
                third_parameter_buffer[position_in_parameter_buffer] = current_char;
                third_parameter_buffer[position_in_parameter_buffer + 1] = '\0';
                break;
        }

        position_in_parameter_buffer++;
    }

    auto parsed_first_parameter = parse_number<float>(first_parameter_buffer);
    auto parsed_second_parameter = parse_number<float>(second_parameter_buffer);
    auto parsed_third_parameter = parse_number<float>(third_parameter_buffer);

    ESP_LOGI(TAG, "First Parameter: %f", parsed_first_parameter);
    ESP_LOGI(TAG, "Second Parameter: %f", parsed_second_parameter);
    ESP_LOGI(TAG, "Third Parameter: %f", parsed_third_parameter);

    switch (this->current_command_)
    {
        // Read Commands
        case EZO_PMP_COMMAND_READ_DOSING:  // Page 54
            if (parsed_third_parameter.has_value())
                this->is_dosing_ = parsed_third_parameter.value_or(0) == 1;

            if (parsed_second_parameter.has_value() && this->last_volume_requested_)
                this->last_volume_requested_ = (parsed_second_parameter.value_or(0));

            if (!this->is_dosing_ && !this->is_paused_)
                // If pump is not paused and not dispensing
                if (this->dosing_mode_ != "" && this->dosing_mode_ != DOSING_MODE_NONE)
                    this->dosing_mode_ = (DOSING_MODE_NONE);
        break;

        case EZO_PMP_COMMAND_READ_SINGLE_REPORT:  // Single Report (page 53)
            if (parsed_first_parameter.has_value() && (bool) this->current_volume_dosed_)
                this->current_volume_dosed_ = (parsed_first_parameter.value_or(0));
            break;

        case EZO_PMP_COMMAND_READ_MAX_FLOW_RATE:  // Constant Flow Rate (page 57)
            if (parsed_second_parameter.has_value() && this->max_flow_rate_)
                this->max_flow_rate_ = (parsed_second_parameter.value_or(0));
            break;

        case EZO_PMP_COMMAND_READ_PAUSE_STATUS:  // Pause (page 61)
            if (parsed_second_parameter.has_value())
                this->is_paused_ = parsed_second_parameter.value_or(0) == 1;
            break;

        case EZO_PMP_COMMAND_READ_TOTAL_VOLUME_DOSED:  // Total Volume Dispensed (page 64)
            if (parsed_second_parameter.has_value() && this->total_volume_dosed_)
                this->total_volume_dosed_ = (parsed_second_parameter.value_or(0));
            break;

        case EZO_PMP_COMMAND_READ_ABSOLUTE_TOTAL_VOLUME_DOSED:  // Total Volume Dispensed (page 64)
            if (parsed_second_parameter.has_value() && this->absolute_total_volume_dosed_)
                this->absolute_total_volume_dosed_ = (parsed_second_parameter.value_or(0));
            break;

        case EZO_PMP_COMMAND_READ_CALIBRATION_STATUS:  // Calibration (page 65)
            if (parsed_second_parameter.has_value() && this->calibration_status_ != "")
                if (parsed_second_parameter.value_or(0) == 1)
                    this->calibration_status_ = ("Fixed Volume");
                else if (parsed_second_parameter.value_or(0) == 2)
                    this->calibration_status_ = ("Volume/Time");
                else if (parsed_second_parameter.value_or(0) == 3)
                    this->calibration_status_ = ("Fixed Volume & Volume/Time");
                else
                    this->calibration_status_ = ("Uncalibrated");
            break;

        case EZO_PMP_COMMAND_READ_PUMP_VOLTAGE:  // Pump Voltage (page 67)
            if (parsed_second_parameter.has_value() && this->pump_voltage_)
                this->pump_voltage_ = (parsed_second_parameter.value_or(0));
            break;

        // Non-Read Commands

        case EZO_PMP_COMMAND_DOSE_VOLUME:  // Volume Dispensing (page 55)
            if (this->dosing_mode_ != "" && this->dosing_mode_ != DOSING_MODE_VOLUME)
                this->dosing_mode_ = (DOSING_MODE_VOLUME);
            break;

        case EZO_PMP_COMMAND_DOSE_VOLUME_OVER_TIME:  // Dose over time (page 56)
            if (this->dosing_mode_ != "" && this->dosing_mode_ != DOSING_MODE_VOLUME_OVER_TIME)
                this->dosing_mode_ = (DOSING_MODE_VOLUME_OVER_TIME);
            break;

        case EZO_PMP_COMMAND_DOSE_WITH_CONSTANT_FLOW_RATE:  // Constant Flow Rate (page 57)
            if (this->dosing_mode_ != "" && this->dosing_mode_ != DOSING_MODE_CONSTANT_FLOW_RATE)
                this->dosing_mode_ = (DOSING_MODE_CONSTANT_FLOW_RATE);
            break;

        case EZO_PMP_COMMAND_DOSE_CONTINUOUSLY:  // Continuous Dispensing (page 54)
            if (this->dosing_mode_ != "" && this->dosing_mode_ != DOSING_MODE_CONTINUOUS)
                this->dosing_mode_ = (DOSING_MODE_CONTINUOUS);
            break;

        case EZO_PMP_COMMAND_STOP_DOSING:  // Stop (page 62)
            this->is_paused_ = false;
            if (this->dosing_mode_ != "" && this->dosing_mode_ != DOSING_MODE_NONE)
                this->dosing_mode_ = (DOSING_MODE_NONE);
            break;

        case EZO_PMP_COMMAND_EXEC_ARBITRARY_COMMAND_ADDRESS:
            ESP_LOGI(TAG, "Arbitrary Command Response: %s", (char *) response_buffer);
            break;

        case EZO_PMP_COMMAND_CLEAR_CALIBRATION:         // Clear Calibration (page 65)
        case EZO_PMP_COMMAND_PAUSE_DOSING:              // Pause (page 61)
        case EZO_PMP_COMMAND_SET_CALIBRATION_VOLUME:    // Set Calibration Volume (page 65)
        case EZO_PMP_COMMAND_CLEAR_TOTAL_VOLUME_DOSED:  // Clear Total Volume Dosed (page 64)
        case EZO_PMP_COMMAND_FIND:                      // Find (page 52)
            // Nothing to do here
            break;

        case EZO_PMP_COMMAND_TYPE_READ:
        case EZO_PMP_COMMAND_NONE:
        case EZO_PMP_COMMAND_CUSTOM:
        default:
            ESP_LOGE(TAG, "Unsupported command received: %d", this->current_command_);
            return;
    }

    this->clear_current_command_();
}
void WaterQuality::send_next_command_()
{
    this->set_i2c_address(EZOPMP_I2C_ADDRESS);
    if (this->is_failed())
        return;

    int wait_time_for_command = 400;  // milliseconds
    uint8_t command_buffer[21];
    int command_buffer_length = 0;

    this->pop_next_command_();  // this->next_command will be updated.

    switch (this->next_command_)
    {
        // Read Commands
        case EZO_PMP_COMMAND_READ_DOSING:  // Page 54
            command_buffer_length = sprintf((char *) command_buffer, "D,?");
            break;

        case EZO_PMP_COMMAND_READ_SINGLE_REPORT:  // Single Report (page 53)
            command_buffer_length = sprintf((char *) command_buffer, "R");
            break;

        case EZO_PMP_COMMAND_READ_MAX_FLOW_RATE:
            command_buffer_length = sprintf((char *) command_buffer, "DC,?");
            break;

        case EZO_PMP_COMMAND_READ_PAUSE_STATUS:
            command_buffer_length = sprintf((char *) command_buffer, "P,?");
            break;

        case EZO_PMP_COMMAND_READ_TOTAL_VOLUME_DOSED:
            command_buffer_length = sprintf((char *) command_buffer, "TV,?");
            break;

        case EZO_PMP_COMMAND_READ_ABSOLUTE_TOTAL_VOLUME_DOSED:
            command_buffer_length = sprintf((char *) command_buffer, "ATV,?");
            break;

        case EZO_PMP_COMMAND_READ_CALIBRATION_STATUS:
            command_buffer_length = sprintf((char *) command_buffer, "Cal,?");
            break;

        case EZO_PMP_COMMAND_READ_PUMP_VOLTAGE:
            command_buffer_length = sprintf((char *) command_buffer, "PV,?");
            break;

        // Non-Read Commands

        case EZO_PMP_COMMAND_FIND:  // Find (page 52)
            command_buffer_length = sprintf((char *) command_buffer, "Find");
            wait_time_for_command = 60000;  // This command will block all updates for a minute
            break;

        case EZO_PMP_COMMAND_DOSE_CONTINUOUSLY:  // Continuous Dispensing (page 54)
            command_buffer_length = sprintf((char *) command_buffer, "D,*");
            break;

        case EZO_PMP_COMMAND_CLEAR_TOTAL_VOLUME_DOSED:  // Clear Total Volume Dosed (page 64)
            command_buffer_length = sprintf((char *) command_buffer, "Clear");
            break;

        case EZO_PMP_COMMAND_CLEAR_CALIBRATION:  // Clear Calibration (page 65)
            command_buffer_length = sprintf((char *) command_buffer, "Cal,clear");
            break;

        case EZO_PMP_COMMAND_PAUSE_DOSING:  // Pause (page 61)
            command_buffer_length = sprintf((char *) command_buffer, "P");
            break;

        case EZO_PMP_COMMAND_STOP_DOSING:  // Stop (page 62)
            command_buffer_length = sprintf((char *) command_buffer, "X");
            break;

        // Non-Read commands with parameters

        case EZO_PMP_COMMAND_DOSE_VOLUME:  // Volume Dispensing (page 55)
            command_buffer_length = sprintf((char *) command_buffer, "D,%0.1f", this->next_command_volume_);
            break;

        case EZO_PMP_COMMAND_DOSE_VOLUME_OVER_TIME:  // Dose over time (page 56)
            command_buffer_length = sprintf((char *) command_buffer, "D,%0.1f,%i", this->next_command_volume_, this->next_command_duration_);
            break;

        case EZO_PMP_COMMAND_DOSE_WITH_CONSTANT_FLOW_RATE:  // Constant Flow Rate (page 57)
            command_buffer_length = sprintf((char *) command_buffer, "DC,%0.1f,%i", this->next_command_volume_, this->next_command_duration_);
            break;

        case EZO_PMP_COMMAND_SET_CALIBRATION_VOLUME:  // Set Calibration Volume (page 65)
            command_buffer_length = sprintf((char *) command_buffer, "Cal,%0.2f", this->next_command_volume_);
            break;

        case EZO_PMP_COMMAND_CHANGE_I2C_ADDRESS:  // Change I2C Address (page 73)
            command_buffer_length = sprintf((char *) command_buffer, "I2C,%i", this->next_command_duration_);
            break;

        case EZO_PMP_COMMAND_EXEC_ARBITRARY_COMMAND_ADDRESS:  // Run an arbitrary command
            command_buffer_length = sprintf((char *) command_buffer, this->arbitrary_command_, this->next_command_duration_);
            ESP_LOGI(TAG, "Sending arbitrary command: %s", (char *) command_buffer);
            break;

        case EZO_PMP_COMMAND_TYPE_READ:
        case EZO_PMP_COMMAND_NONE:
        case EZO_PMP_COMMAND_CUSTOM:
            command_buffer_length = sprintf((char *) command_buffer, this->custom_.c_str());
            ESP_LOGI(TAG, "Sending custom command: %s", (char *) command_buffer);
            break;

        default:
            ESP_LOGE(TAG, "Unsupported command received: %d", this->next_command_);
            return;
    }
    // Send command
    if (this->current_command_ != this->next_command_)
        ESP_LOGI(TAG, "Sending command to device: %s", (char *) command_buffer);

    this->write(command_buffer, command_buffer_length);

    this->current_command_ = this->next_command_;
    this->next_command_ = EZO_PMP_COMMAND_NONE;
    this->is_waiting_ = true;
    this->start_time_ = millis();
    this->wait_time_ = wait_time_for_command;
}
void WaterQuality::pop_next_command_()
{
    if (this->next_command_queue_length_ <= 0)
    {
        ESP_LOGE(TAG, "Tried to dequeue command from empty queue");
        this->next_command_ = EZO_PMP_COMMAND_NONE;
        this->next_command_volume_ = 0;
        this->next_command_duration_ = 0;
        return;
    }

    // Read from Head
    this->next_command_ = this->next_command_queue_[this->next_command_queue_head_];
    this->next_command_volume_ = this->next_command_volume_queue_[this->next_command_queue_head_];
    this->next_command_duration_ = this->next_command_duration_queue_[this->next_command_queue_head_];

    // Move positions
    next_command_queue_head_++;
    if (next_command_queue_head_ >= 10)
        next_command_queue_head_ = 0;

    next_command_queue_length_--;
}
uint16_t WaterQuality::peek_next_command_()
{
    if (this->next_command_queue_length_ <= 0)
        return EZO_PMP_COMMAND_NONE;

    return this->next_command_queue_[this->next_command_queue_head_];
}
void WaterQuality::queue_command_(uint16_t command, float volume, int duration, bool should_schedule)
{
    if (!should_schedule)
        return;

    if (this->next_command_queue_length_ >= 10)
    {
        ESP_LOGE(TAG, "Tried to queue command '%d' but queue is full", command);
        return;
    }

    this->next_command_queue_[this->next_command_queue_last_] = command;
    this->next_command_volume_queue_[this->next_command_queue_last_] = volume;
    this->next_command_duration_queue_[this->next_command_queue_last_] = duration;

    ESP_LOGV(TAG, "Queue command '%d' in position '%d'", command, next_command_queue_last_);

    // Move positions
    next_command_queue_last_++;
    if (next_command_queue_last_ >= 10)
        next_command_queue_last_ = 0;

    next_command_queue_length_++;
}
void WaterQuality::exec_arbitrary_command(const std::basic_string<char> &command)
{
    this->arbitrary_command_ = command.c_str();
    this->queue_command_(EZO_PMP_COMMAND_EXEC_ARBITRARY_COMMAND_ADDRESS, 0, 0, true);
}
void WaterQuality::custom_command(std::string custom)
{
    // int wait_time_for_command = 400;  // milliseconds
    // uint8_t command_buffer[21];
    // int command_buffer_length = 0;
    // command_buffer_length = sprintf((char *) command_buffer, custom.c_str());

    // clear_current_command_();

    // Send command
    this->custom_ = custom.c_str();
    // this->queue_command_(EZO_PMP_COMMAND_CUSTOM, 0, 0, true);

    // ESP_LOGI(TAG, "Sending command to device: %s", custom.c_str());
    // this->write(command_buffer, command_buffer_length);
    
    // this->current_command_ = this->next_command_;
    // this->next_command_ = EZO_PMP_COMMAND_NONE;
    // this->is_waiting_ = true;
    // this->start_time_ = millis();
    // this->wait_time_ = wait_time_for_command;
    
    // this->read_command_result_();
}

void WaterQuality::EZOPMP_Read()
{
    this->set_i2c_address(EZOPMP_I2C_ADDRESS);
    if (this->is_failed())
        return;

    uint8_t response_buffer[21] = {'\0'};
    response_buffer[0] = 0;

    //if (this->is_waiting_ /*&& millis() - this->start_time_ >= 380*/)
    // {
        // ESP_LOGE(TAG, "wait time = %d", millis() - this->start_time_);
        // this->is_waiting_ = 0;

this->read_bytes_raw(response_buffer, 20);
        // if (!this->read_bytes_raw(response_buffer, 20);)
        //     return;

        for (size_t i = 0; i < 21; i++)
            if (this->command2_[i] != response_buffer[i] /*&& response_buffer[0] <= 1*/)
            {
                ESP_LOGE(TAG, "wait time = %d", millis() - this->start_time_);
                ESP_LOGE(TAG, "response_buffer[%d] = %d", i, response_buffer[i]);
                ESP_LOGI(TAG, "Read Response from device: %s", (char *) response_buffer);
        
                this->command2_[i] = response_buffer[i];
            }
        this->start_time_ = millis();
    // }
}
void WaterQuality::EZOPMP_Write()
{
    this->set_i2c_address(EZOPMP_I2C_ADDRESS);
    if (this->is_failed())
        return;

    uint8_t command_buffer[21];
    int command_buffer_length = 0;
    command_buffer_length = sprintf((char *) command_buffer, this->custom_.c_str());
    
    if (command_buffer_length > 0)
    {
        this->write(command_buffer, command_buffer_length);
        ESP_LOGI(TAG, "Sending command to device: %s", this->custom_.c_str());
        
        this->is_waiting_ = 1;
        this->start_time_ = millis();
        this->custom_ = "";
    }
}
void WaterQuality::EZOPMP_Driver(float volume[])
{
    this->set_i2c_address(EZOPMP_I2C_ADDRESS);
    if (this->is_failed())
        return;

    // uint8_t model = 0;
    
    // for (size_t i = 0; i < 6; i++)
    // {
    //     if (Pump_Model[i] != 0)
    //         model++;
    // }

    // if (model > 0)
    //     for (size_t i = 0; i < model; i++)
    //     {
    //         this->set_i2c_address(EZOPMP_I2C_ADDRESS);
    //         if (!this->is_failed())
    //             change_i2c_address(EZOPMP_I2C_ADDRESS + i + 1);
    //     }

    // EZOPMP_loop();
    // EZOPMP_update();
    EZOPMP_Read();
    EZOPMP_Write();

    uint8_t command[50] = {0}, len = 20;
    
    // this->read_bytes_raw(command, len);
    // if (this->command_[0] != command[0] /*&& command[0] <= 1*/)
    //     ESP_LOGI(TAG, "command: %s", (char *) command);
    // for (size_t i = 0; i < len; i++)
    //     if (this->command_[i] != command[i])
    //     {
    //         this->command_[i] = command[i];
    //         ESP_LOGI(TAG,"read[%d] = %d", i, command[i]);
    //     }
        
    for (size_t i = 0; i < 6; i++)
    {
        // if (this->volume_[i] != volume[i])
        // {
        //     if (volume[i] > 0)
        //     {
        //         dose_volume(volume[i]);
        //         ESP_LOGI(TAG,"volume[%d] = %f", i, volume[i]);
        //     }
        //     else if (volume[i] == 0)
        //         stop_dosing();
                
        //     this->volume_[i] == volume[i];
        // }

        // ESP_LOGI(TAG,"get_is_dosing = %d", get_is_dosing());
        
        if (get_is_dosing())
        {
            ESP_LOGI(TAG,"total_volume_dosed_[%d] = %f", i, get_total_volume_dosed());
            ESP_LOGI(TAG,"absolute_total_volume_dosed_[%d] = %f", i, get_absolute_total_volume_dosed());
        }
        
        if (volume[i] > 0 && !get_is_dosing())
        {
            // dose_volume(volume[i]);
            this->custom_command("D," + std::to_string(volume[i]));
        }
        else if (volume[i] == 0 && get_is_dosing())
        {
            // stop_dosing();
            this->custom_command("X");
            ESP_LOGI(TAG,"Pump%d Stopped", i + 1);
        }
    }
}
void WaterQuality::EZOPMP_loop()
{
    // If we are not waiting for anything and there is no command to be sent, return
    if (!this->is_waiting_ && this->peek_next_command_() == EZO_PMP_COMMAND_NONE)
        return;

    // If we are not waiting for anything and there IS a command to be sent, do it.
    if (!this->is_waiting_ && this->peek_next_command_() != EZO_PMP_COMMAND_NONE)
        this->send_next_command_();

    // If we are waiting for something but it isn't ready yet, then return
    if (this->is_waiting_ && millis() - this->start_time_ < this->wait_time_)
        return;

    // We are waiting for something and it should be ready.
    this->read_command_result_();
}
void WaterQuality::EZOPMP_update()
{
    if (this->is_waiting_)
        return;

    if (this->is_first_read_)
    {
        this->queue_command_(EZO_PMP_COMMAND_READ_CALIBRATION_STATUS, 0, 0, true);
        this->queue_command_(EZO_PMP_COMMAND_READ_MAX_FLOW_RATE, 0, 0, (bool) this->max_flow_rate_);
        this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
        this->queue_command_(EZO_PMP_COMMAND_READ_TOTAL_VOLUME_DOSED, 0, 0, (bool) this->total_volume_dosed_);
        this->queue_command_(EZO_PMP_COMMAND_READ_ABSOLUTE_TOTAL_VOLUME_DOSED, 0, 0, (bool) this->absolute_total_volume_dosed_);
        this->queue_command_(EZO_PMP_COMMAND_READ_PAUSE_STATUS, 0, 0, true);
        this->is_first_read_ = false;
    }

    if (!this->is_waiting_ && this->peek_next_command_() == EZO_PMP_COMMAND_NONE)
    {
        this->queue_command_(EZO_PMP_COMMAND_READ_DOSING, 0, 0, true);

        if (this->is_dosing_)
        {
            this->queue_command_(EZO_PMP_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
            this->queue_command_(EZO_PMP_COMMAND_READ_TOTAL_VOLUME_DOSED, 0, 0, (bool) this->total_volume_dosed_);
            this->queue_command_(EZO_PMP_COMMAND_READ_ABSOLUTE_TOTAL_VOLUME_DOSED, 0, 0, (bool) this->absolute_total_volume_dosed_);
        }

        this->queue_command_(EZO_PMP_COMMAND_READ_PUMP_VOLTAGE, 0, 0, (bool) this->pump_voltage_);
    }
    else
        ESP_LOGV(TAG, "Not Scheduling new Command during update()");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void EC10() {/*DFRobot_EC10 ec;*/}
// void EC() {DFRobot_EC ec;}
    
    // DFRobot_EC ec;
    // DFRobot_PH ph;

}  // namespace water_quality
}  // namespace esphome