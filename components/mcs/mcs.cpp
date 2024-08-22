#include "mcs.h"
#include "mcs_i2c.h"
#include "mcs_digital.h"

namespace esphome {
namespace mcs {

    MCS_Digital dig;
    // uart::UARTDevice *uart;
    // HardwareSerial& odrive_serial = Serial1;
    
    SoftwareSerial odrive_serial(9, 10);
    ODriveArduino odrive(odrive_serial);

void EEPROM_Write(int address, float value)
{
    byte *data = (byte*)&(value);
    for (int i = 0; i < sizeof(value); i++)
        EEPROM.write(address + i, data[i]);
}
float EEPROM_Read(int address)
{
    float value = 0.0;
    byte *data = (byte*)&(value);
    for (int i = 0; i < sizeof(value); i++)
        data[i] = EEPROM.read(address + i);
    return value;
}
void EEPROM_Setup()
{
    uint8_t digital = EEPROM_Read(LED_L_ADDR); // Load the value of the digital from the EEPROM
    bool digital_status[] = {0};
    for (uint8_t i = 0; i < 20; i++)
    {
        if (digital > i)
            digital_status[i] = 1;
        else break;
    }
    dig.set_Digital_Output(digital_status);
}

uint8_t del = 40;
uint8_t state = 1;
uint8_t q = 0;
bool digital1[16] = {0};
bool digital2[4] = {0};
uint32_t time = 0;
void MCS::start()
{
    if (millis() - time >= del)
    {
        time = millis();
        switch (state)
        {
            case 1:
                if (q < 20)
                {
                    if (q < 16)
                    {
                        digital1[q] = 1;
                        if (q > 0)
                        {
                            digital1[q - 1] = 0;
                        }
                    }
                    else
                    {
                        digital2[q % 16] = 1;
                        if (q > 16)
                        {
                            digital2[q % 16 - 1] = 0;
                        }
                        else
                        {
                            digital1[q - 1] = 0;
                        }
                    }
                    q++;
                }
                else
                {
                    state = 2; // Sonraki işlem
                    q = 18;
                }
                
                this->set_i2c_address(LEFT_ADDRESS1);
                if (this->is_failed())
                    return;
                
                MCP23017_Write(digital1, 1);

                this->set_i2c_address(LEFT_ADDRESS2);
                if (this->is_failed())
                    return;

                MCP23017_Write(digital2, 2);

                this->set_i2c_address(RIGHT_ADDRESS1);
                if (this->is_failed())
                    return;
                
                MCP23017_Write(digital1, 1);

                this->set_i2c_address(RIGHT_ADDRESS2);
                if (this->is_failed())
                    return;

                MCP23017_Write(digital2, 2);

                break;
                
            case 2:
                if (q >= 0)
                {
                    if (q >= 16)
                    {
                        digital2[q % 16] = 1;
                        digital2[(q + 1) % 16] = 0;
                    }
                    else
                    {
                        digital1[q] = 1;
                        if (q < 15)
                        {
                        digital1[q + 1] = 0;
                        }
                        else
                        {
                        digital2[(q + 1) % 16] = 0;
                        }
                    }
                    if (q == 0)
                        state = 0; // İşlemi bitir
                    else
                        q--;
                }

                this->set_i2c_address(LEFT_ADDRESS1);
                if (this->is_failed())
                    return;
                
                MCP23017_Write(digital1, 1);

                this->set_i2c_address(LEFT_ADDRESS2);
                if (this->is_failed())
                    return;

                MCP23017_Write(digital2, 2);

                this->set_i2c_address(RIGHT_ADDRESS1);
                if (this->is_failed())
                    return;
                
                MCP23017_Write(digital1, 1);

                this->set_i2c_address(RIGHT_ADDRESS2);
                if (this->is_failed())
                    return;

                MCP23017_Write(digital2, 2);

                break;
        }
    }
}

void MCS::setup()
{
    // EEPROM.begin(MCS_EEPROM_SIZE);
    // EEPROM_Setup();
    MCP23017_Setup(BUTTON_ADDRESS1);
    MCP23017_Setup(BUTTON_ADDRESS2);
    MCP23017_Setup(LEFT_ADDRESS1);
    MCP23017_Setup(LEFT_ADDRESS2);
    MCP23017_Setup(RIGHT_ADDRESS1);
    MCP23017_Setup(RIGHT_ADDRESS2);
    
    odrive_serial.begin(115200);
}
void MCS::dump_config()
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  I2C

    ESP_LOGCONFIG(TAG, "");
    ESP_LOGCONFIG(TAG, "I2C:");
    
    LOG_I2C_DEVICE(this);
    // LOG_UPDATE_INTERVAL(this);
    if (this->is_failed())
    {
        ESP_LOGE(TAG, "  Communication failed!");
        // return;
    }
    else
        ESP_LOGI(TAG, "  Communication Successfulled!");
        
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  UART
        
    ESP_LOGCONFIG(TAG, "UART:");
    this->check_uart_settings(115200);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TCA9548

    Wire.beginTransmission(TCA9548_ADDRESS);
    if(!Wire.endTransmission())
    {
        ESP_LOGCONFIG(TAG, "  TCA9548:");

        for (uint8_t t=0; t<8; t++)
        {
            tcaselect(t);
            ESP_LOGI(TAG, "    Channel %d:", t);

            for (uint8_t addr = 0; addr<=127; addr++) 
            {
                if (addr == TCA9548_ADDRESS) continue;

                Wire.beginTransmission(addr);
                if (!Wire.endTransmission()) 
                {
                    if (addr < 16)
                        ESP_LOGI(TAG, "      Found I2C 0x0%x",addr);
                    else
                        ESP_LOGI(TAG, "      Found I2C 0x%x",addr);
                }
                else if(Wire.endTransmission() == 4)
                    ESP_LOGE(TAG, "      Found the same I2C 0x%x",addr);
            }
        }
    }
    else
    {
        ESP_LOGCONFIG(TAG, "  I2C Devices:");

        for (uint8_t addr = 0; addr<=127; addr++) 
        {
            Wire.beginTransmission(addr);
            if (!Wire.endTransmission()) 
            {
                if (addr < 16)
                    ESP_LOGI(TAG, "    Found I2C 0x0%x",addr);
                else
                    ESP_LOGI(TAG, "    Found I2C 0x%x",addr);
            }
            else if(Wire.endTransmission() == 4)
                ESP_LOGE(TAG, "    Found the same I2C 0x%x",addr);
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ESP_LOGCONFIG(TAG, "Digital:");

uint8_t digital = EEPROM_Read(LED_L_ADDR);; // Load the value of the digital from the EEPROM
ESP_LOGI(TAG, "  Value: %d", digital);
}

void MCS::loop()
{
    if (state)
        start();
}
void MCS::update()
{
    bool d[20];

    dig.Digital_Output_Driver(d);
    MCP23017_Driver(d);
}

void MCS::version(const uint8_t ver)
{
}
void MCS::digital_out(std::vector<bool> &dout)
{
    bool* dout_ = dig.get_Digital_Output();
    std::vector<bool> d(dout_, dout_ + 20);

    if (d != dout)
    {
        dout_[0] = 1;
        uint8_t digital = 1;
        ESP_LOGD(TAG, "DigOut_Status[0] = %d", dout_[0]);

        for (uint8_t i = 1; i < 20; i++)
        {
            dout_[i] = dout[i];
            ESP_LOGD(TAG, "DigOut_Status[%d] = %d", i, dout_[i]);

            if (dout[i])
                digital++;
        }
        ESP_LOGD(TAG, "digital = %d", digital);
        // EEPROM_Write(LED_L_ADDR, digital); // Store the current value
        // EEPROM.commit();
    }
}
void MCS::digital_out2(uint8_t dout)
{
    bool dout_[20] = {0};
    uint8_t d = dig.get_Digital_Output2();

    if (d != dout)
    {
        d = dout;
        dout_[0] = 1;
        ESP_LOGD(TAG, "DigOut_Status[0] = %d", dout_[0]);

        for (uint8_t i = 1; i < dout; i++)
        {
            dout_[i] = 1;
            ESP_LOGD(TAG, "DigOut_Status[%d] = %d", i, dout_[i]);
        }
        dig.set_Digital_Output(dout_);
        ESP_LOGD(TAG, "digital = %d", dout);
        // EEPROM_Write(LED_L_ADDR, dout); // Store the current value
        // EEPROM.commit();
    }
}

}  // namespace mcs
}  // namespace esphome