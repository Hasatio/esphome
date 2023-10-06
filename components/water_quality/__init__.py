import esphome.codegen as cg
import esphome.config_validation as cv
# from esphome.components import sensor
from esphome.const import CONF_ID

AUTO_LOAD = ["sensor"]
MULTI_CONF = True

component_ns = cg.esphome_ns.namespace("water_quality")
MyComponent = component_ns.class_("MyComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MyComponent)
}).extend(cv.COMPONENT_SCHEMA)

# def to_code(config):
#     var = cg.new_Pvariable(config[CONF_ID])
#     yield cg.register_component(var, config)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add_library("Wire", None)
    cg.add_library("SPI", None)
    cg.add_library("Adafruit BusIO",None)
    cg.add_library("Adafruit ADS1X15", None)
    cg.add_library("Adafruit MCP23017 library", None)
    cg.add_library("Adafruit PWM Servo Driver Library", None)
    