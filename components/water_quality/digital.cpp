#include "mux.h"
#include "digital.h"
#include "water_quality.h"

namespace esphome {
namespace water_quality {

    Mux mux;
    Digital dig;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP23008

    Adafruit_MCP23X08 mcp;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Digital::mcp23008_set()
{
    // tcaselect(0);
    
    if (!mcp.begin_I2C(MCP23008_ADDRESS, &Wire)) 
    {
        ESP_LOGE(TAG,"Failed to initialize MCP23008.");
        // while (1);
    }

    // mcp.pinMode(0, INPUT);
    // mcp.pinMode(1, INPUT);
    // mcp.pinMode(2, INPUT);
    // mcp.pinMode(3, INPUT);
    // mcp.pullUp(0, HIGH);
    // mcp.pullUp(1, HIGH);
    // mcp.pullUp(2, HIGH);
    // mcp.pullUp(3, HIGH);
    // mcp.pinMode(4, OUTPUT);
    // mcp.pinMode(5, OUTPUT);
    // mcp.pinMode(6, OUTPUT);
    // mcp.pinMode(7, OUTPUT);
    // mcp.digitalWrite(4,LOW);
    // mcp.digitalWrite(5,LOW);
    // mcp.digitalWrite(6,LOW);
    // mcp.digitalWrite(7,LOW);
    
    mcp.pinMode(0, INPUT_PULLUP);
    mcp.pinMode(1, INPUT_PULLUP);
    mcp.pinMode(2, INPUT_PULLUP);
    mcp.pinMode(3, INPUT_PULLUP);
    mcp.pinMode(4, OUTPUT);
    mcp.pinMode(5, OUTPUT);
    mcp.pinMode(6, OUTPUT);
    mcp.pinMode(7, OUTPUT);
    mcp.digitalWrite(4,LOW);
    mcp.digitalWrite(5,LOW);
    mcp.digitalWrite(6,LOW);
    mcp.digitalWrite(7,LOW);
}
void Digital::mcp23008()
{
    // tcaselect(0);
    mcp.digitalWrite(4,LOW);
    mcp.digitalWrite(5,LOW);
    mcp.digitalWrite(6,LOW);
    mcp.digitalWrite(7,LOW);

    for(size_t i = 0; i < 4; i++)
    {
        DigIn_Read[i] = mcp.digitalRead(i);
        // ESP_LOGD(TAG,"dig input %d = %d", i, DigIn_Read[i]);
    }

    for(size_t i = 0; i < 4; i++)
    {
        if (DigOut_Status[i] == 1)
        {
            mcp.digitalWrite(i + 4, HIGH);
        }
        else
        { 
            mcp.digitalWrite(i + 4, LOW);
        }
        // ESP_LOGD(TAG,"dig output %d = %d", i, DigOut_Status[i]);
    }
}

void MyComponent::digital_out(std::vector<bool> &dout)
{
    if (dig.DigOut_Status != dout)
    {
        this->DigOut_Status = dout;
        for (size_t i = 0; i < dig.DigOut_Status.size(); i++)
        {
            ESP_LOGD(digital,"DigOut_Status[%d] = %d", i, (int)dig.DigOut_Status[i]);
        }
    }
}

}  // namespace water_quality
}  // namespace esphome