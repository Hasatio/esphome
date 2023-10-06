import esphome.codegen as cg
import esphome.config_validation as cv
# from esphome.components import sensor
from esphome.const import CONF_ID, CONF_DATA

AUTO_LOAD = ["sensor"]
MULTI_CONF = True

CONF_USER_CHARACTERS = "user_characters"

component_ns = cg.esphome_ns.namespace("water_quality")
MyComponent = component_ns.class_("MyComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MyComponent),
    cv.Optional(CONF_USER_CHARACTERS): cv.All(
            cv.ensure_list(
                cv.Schema(
                    {
                        cv.Required(CONF_DATA): cv.All(
                            cv.ensure_list(cv.int_range(min=0, max=31))
                        ),
                    }
                ),
            ),
            cv.Length(max=8),
        ),
}).extend(cv.COMPONENT_SCHEMA)

# def to_code(config):
#     var = cg.new_Pvariable(config[CONF_ID])
#     yield cg.register_component(var, config)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    if CONF_USER_CHARACTERS in config:
        for usr in config[CONF_USER_CHARACTERS]:
            cg.add(var.dat(usr[CONF_DATA]))
    
    cg.add_library("Wire", None)
    cg.add_library("SPI", None)
    cg.add_library("Adafruit BusIO",None)
    cg.add_library("Adafruit ADS1X15", None)
    cg.add_library("Adafruit MCP23017 Arduino Library", None)
    cg.add_library("Adafruit PWM Servo Driver Library", None)
    