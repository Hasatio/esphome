#include "mcs.h"
#include "mcs_i2c.h"

namespace esphome {
namespace mcs {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23017

void MCS::MCP23017_Setup(uint8_t address)
{
    this->set_i2c_address(address);
    if (this->is_failed())
        return;

    ESP_LOGCONFIG(TAG, "Setting up MCP23017...");

    uint8_t iocon;
    if (!this->I2CDevice::read_byte(MCP23017_IOCONA, &iocon))
    {
        this->mark_failed();
        return;
    }

    uint8_t reg_value = 0;
    if (address < LEFT_ADDRESS1)
        for (uint8_t i = 0; i < 8; i++)
            reg_value |= 1 << i; // input
    else
        for (uint8_t i = 0; i < 8; i++)
            reg_value &= ~(1 << i); // output
            

    this->I2CDevice::write_byte(MCP23017_GPIOA, reg_value);
    this->I2CDevice::write_byte(MCP23017_IODIRA, reg_value);
    this->I2CDevice::write_byte(MCP23017_GPPUA, reg_value);
    this->I2CDevice::write_byte(MCP23017_OLATA, reg_value);

    this->I2CDevice::write_byte(MCP23017_GPIOB, reg_value);
    this->I2CDevice::write_byte(MCP23017_IODIRB, reg_value);
    this->I2CDevice::write_byte(MCP23017_GPPUB, reg_value);
    this->I2CDevice::write_byte(MCP23017_OLATB, reg_value);

    if (address == LEFT_ADDRESS1 || address == RIGHT_ADDRESS1)
        this->I2CDevice::write_byte(MCP23017_OLATA, reg_value + 1);


    // Read current output register state
    // if (address == MCP23017_ADDRESS1)
    // {
    // this->read_byte(MCP23017_OLATA, &this->olat_a1_);
    // this->read_byte(MCP23017_OLATB, &this->olat_b_);
    // }
    // else
    // this->read_byte(MCP23017_OLATA, &this->olat_a2_);
    

    // this->write_byte(MCP23017_IPOLA, 0x00);
    // this->write_byte(MCP23017_GPINTENA, 0x00);
    // this->write_byte(MCP23017_DEFVALA, 0x00);
    // this->write_byte(MCP23017_INTCONA, 0x00);

    // this->write_byte(MCP23017_IPOLB, 0x00);
    // this->write_byte(MCP23017_GPINTENB, 0x00);
    // this->write_byte(MCP23017_DEFVALB, 0x00);
    // this->write_byte(MCP23017_INTCONB, 0x00);

    // if (this->open_drain_ints_)
    //     // enable open-drain interrupt pins, 3.3V-safe
    //     this->write_byte(MCP23017_IOCONA, 0x04);
    //     this->write_byte(MCP23017_IOCONB, 0x04);

    // this->write_byte(MCP23017_INTFA, 0x00);
    // this->write_byte(MCP23017_INTCAPA, 0x00);
    
    // this->write_byte(MCP23017_INTFB, 0x00);
    // this->write_byte(MCP23017_INTCAPB, 0x00);

}
void MCS::MCP23017_Read(bool value[])
{
    uint8_t value_;

    this->I2CDevice::read_byte(MCP23017_GPIOA, &value_);

    for (uint8_t i = 0; i < 8; i++)
        value[i] = value_ & (1 << i);

    this->I2CDevice::read_byte(MCP23017_GPIOB, &value_);

    for (uint8_t i = 0; i < 8; i++)
        value[i + 8] = value_ & (1 << i);
}
void MCS::MCP23017_Write(bool value[], uint8_t state)
{
    uint8_t reg_value_a = 0;
    uint8_t reg_value_b = 0;

    switch (state)
    {
        case 1:
            // reg_value_a = this->olat_a1_;
            // reg_value_b = this->olat_b_;

            for (uint8_t i = 0; i < 16; i++)
            {
                if (i < 8)
                    if (value[i])
                        reg_value_a |= 1 << (i);
                    else
                        reg_value_a &= ~(1 << (i));
                else
                    if (value[i])
                        reg_value_b |= 1 << (i % 8);
                    else
                        reg_value_b &= ~(1 << (i % 8));
            }

            // if (reg_value_a != this->olat_a1_)
            // {
                // this->I2CDevice::write_byte(MCP23017_GPIOA, reg_value_a);
                this->I2CDevice::write_byte(MCP23017_OLATA, reg_value_a);
                this->olat_a1_ = reg_value_a;
            // }
            // if (reg_value_b != this->olat_b_)
            // {
                // this->I2CDevice::write_byte(MCP23017_GPIOB, reg_value_b);
                this->I2CDevice::write_byte(MCP23017_OLATB, reg_value_b);
                this->olat_b_ = reg_value_b;
            // }
            break;

        case 2:
            // reg_value_a = this->olat_a2_;

            for (uint8_t i = 0; i < 4; i++)
            {
                if (value[i])
                    reg_value_a |= 1 << (i);
                else
                    reg_value_a &= ~(1 << (i));
            }

            // if (reg_value_a != this->olat_a2_)
            // {
                // this->I2CDevice::write_byte(MCP23017_GPIOA, reg_value_a);
                this->I2CDevice::write_byte(MCP23017_OLATA, reg_value_a);
                this->olat_a2_ = reg_value_a;
            // }
            break;
    }
}
void MCS::MCP23017_Driver(bool digital[])
{
    bool button1[16] = {0};
    bool button2[16] = {0};

    this->I2CDevice::set_i2c_address(BUTTON_ADDRESS1);
    if (this->is_failed())
        return;

    MCP23017_Read(button1);

    this->I2CDevice::set_i2c_address(BUTTON_ADDRESS2);
    if (this->is_failed())
        return;

    MCP23017_Read(button2);

    uint8_t joystick = 0;
    if (!button2[4])
        joystick = 1;
    else if (!button2[5]){
        joystick = 2;
            for (uint8_t i = 0; i < 16; i++)
            ESP_LOGD(TAG,"button1[%d]: %d", i, button1[i]);}

    bool led1[16] = {0};
    bool led2[4] = {0};
    bool left = 0, right = 0;

    switch (joystick)
    {
        case 1:
            for (uint8_t i = 0; i < 16; i++)
                if (!button1[i])
                {
                    led1[i] = 1;
                    right = 1;
                    break;
                }
            if (!right)
                for (uint8_t i = 0; i < 4; i++)
                    if (!button2[i])
                    {
                        led2[i] = 1;
                        right = 1;
                        break;
                    }
            break;
        
        case 2:
            for (uint8_t i = 0; i < 16; i++)
                if (!button1[i])
                {
                    led1[i] = 1;
                    left = 1;
            ESP_LOGD(TAG,"led1[%d]: %d", i, led1[i]);
                    break;
                }
            if (!left)
                for (uint8_t i = 0; i < 4; i++)
                    if (!button2[i])
                    {
                        led2[i] = 1;
                        left = 1;
            ESP_LOGD(TAG,"led2[%d]: %d", i, led2[i]);
                        break;
                    }
            break;
        
        default:
            break;
    }
    
    if (left)
    {
        this->I2CDevice::set_i2c_address(LEFT_ADDRESS1);
        if (this->is_failed())
            return;
        
        MCP23017_Write(led1, 1);

        this->I2CDevice::set_i2c_address(LEFT_ADDRESS2);
        if (this->is_failed())
            return;

        MCP23017_Write(led2, 2);
    }
    else if (right)
    {   
        this->I2CDevice::set_i2c_address(RIGHT_ADDRESS1);
        if (this->is_failed())
            return;
    
        MCP23017_Write(led1, 1);

        this->I2CDevice::set_i2c_address(RIGHT_ADDRESS2);
        if (this->is_failed())
            return;
        
        MCP23017_Write(led2, 2);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace mcs
}  // namespace esphome