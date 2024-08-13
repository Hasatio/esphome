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
    if (!this->read_byte(MCP23017_IOCON, &iocon))
    {
        this->mark_failed();
        return;
    }

    // uint8_t reg_value = 0;
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     reg_value |= 1 << i; // input
    // }
    for (uint8_t i = 0; i < 16; i++)
    {
        reg_value &= ~(1 << i); // output
    }

    this->write_byte(MCP23017_GPIOA, reg_value);
    this->write_byte(MCP23017_IODIRA, reg_value);
    this->write_byte(MCP23017_GPPUA, reg_value);
    this->write_byte(MCP23017_OLATA, reg_value);

    this->write_byte(MCP23017_GPIOB, reg_value);
    this->write_byte(MCP23017_IODIRB, reg_value);
    this->write_byte(MCP23017_GPPUB, reg_value);
    this->write_byte(MCP23017_OLATB, reg_value);

    // Read current output register state
    this->read_byte(MCP23017_OLATA, &this->olat_a_);
    this->read_byte(MCP23017_OLATB, &this->olat_b_);


    this->write_byte(MCP23017_IPOLA, 0x00);
    this->write_byte(MCP23017_GPINTENA, 0x00);
    this->write_byte(MCP23017_DEFVALA, 0x00);
    this->write_byte(MCP23017_INTCONA, 0x00);

    this->write_byte(MCP23017_IPOLB, 0x00);
    this->write_byte(MCP23017_GPINTENB, 0x00);
    this->write_byte(MCP23017_DEFVALB, 0x00);
    this->write_byte(MCP23017_INTCONB, 0x00);

    if (this->open_drain_ints_)
        // enable open-drain interrupt pins, 3.3V-safe
        this->write_byte(MCP23017_IOCONA, 0x04);
        this->write_byte(MCP23017_IOCONB, 0x04);

    this->write_byte(MCP23017_INTFA, 0x00);
    this->write_byte(MCP23017_INTCAPA, 0x00);
    
    this->write_byte(MCP23017_INTFB, 0x00);
    this->write_byte(MCP23017_INTCAPB, 0x00);
}
uint8_t MCS::MCP23017_Read()
{
    uint8_t value;
    this->read_byte(MCP23017_GPIO, &value);

    return value;
}
void MCS::MCP23017_Write(bool value[])
{
    uint8_t reg_value = this->olat_;

    for (uint8_t i = 0; i < 16; i++)
    {
        // uint8_t olat_;
        // this->read_byte(MCP23017_OLAT, &this->olat_);

        if (value[i])
            reg_value |= 1 << (i + 4);
        else
            reg_value &= ~(1 << (i + 4));
    }

    if (reg_value != this->olat_)
    {
        // this->write_byte(MCP23017_GPIO, reg_value);
        this->write_byte(MCP23017_OLAT, reg_value);
        this->olat_ = reg_value;
    }
}
void MCS::MCP23017_pin_interrupt_mode(uint8_t pin, MCP23017_InterruptMode interrupt_mode)
{
    uint8_t gpinten = MCP23017_GPINTEN;
    uint8_t intcon = MCP23017_INTCON;
    uint8_t defval = MCP23017_DEFVAL;

    // switch (interrupt_mode)
    // {
    //   case MCP23017_CHANGE:
    //     this->MCP23017_update_reg(pin, true, gpinten);
    //     this->MCP23017_update_reg(pin, false, intcon);
    //     break;
    //   case MCP23017_RISING:
    //     this->MCP23017_update_reg(pin, true, gpinten);
    //     this->MCP23017_update_reg(pin, true, intcon);
    //     this->MCP23017_update_reg(pin, true, defval);
    //     break;
    //   case MCP23017_FALLING:
    //     this->MCP23017_update_reg(pin, true, gpinten);
    //     this->MCP23017_update_reg(pin, true, intcon);
    //     this->MCP23017_update_reg(pin, false, defval);
    //     break;
    //   case MCP23017_NO_INTERRUPT:
    //     this->MCP23017_update_reg(pin, false, gpinten);
    //     break;
    // }
}
void MCS::MCP23017_Driver(bool digital[])
{
    bool digital1[] = {0};
    for (uint8_t i = 0; i < 16; i++)
        digital1[i] = digital[i];

    this->set_i2c_address(MCP23017_ADDRESS1);
    if (this->is_failed())
        return;
    
    MCP23017_Write(digital1);

    bool digital2[] = {0};
    for (uint8_t i = 0; i < 16; i++)
        digital2[i] = digital[i + 16];

    this->set_i2c_address(MCP23017_ADDRESS2);
    if (this->is_failed())
        return;
    
    MCP23017_Write(digital2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}  // namespace mcs
}  // namespace esphome