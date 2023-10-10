import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, 
    CONF_DATA,
    CONF_CUSTOM,
) 

AUTO_LOAD = ["sensor"]
MULTI_CONF = True

CONF_USER_CHARACTERS = "user_characters"
CONF_PUMP_TOTAL = "pump_total"
CONF_PUMP_STATUS = "pump_status"
CONF_ANALOG_OUTPUT = "analog_output"

UNIT_MILILITER = "ml"
UNIT_MILILITERS_PER_MINUTE = "ml/min"

component_ns = cg.esphome_ns.namespace("water_quality")
MyComponent = component_ns.class_("MyComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MyComponent),
    # cv.Optional(CONF_USER_CHARACTERS): cv.All(
    #         cv.ensure_list(
    #             cv.Schema(
    #                 {
    #                     cv.Required(CONF_DATA): cv.All(
    #                         cv.ensure_list(cv.uint8_t)
    #                     ),
    #                 }
    #             ),
    #         ),
    #         cv.Length(max=8),
    #     ),
    # cv.Optional(CONF_CUSTOM): cv.ensure_list(cv.uint8_t),
}).extend(cv.COMPONENT_SCHEMA)

# def to_code(config):
#     var = cg.new_Pvariable(config[CONF_ID])
#     yield cg.register_component(var, config)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # if CONF_USER_CHARACTERS in config:
    #     for usr in config[CONF_USER_CHARACTERS]:
    #         cg.add(var.set_user_defined_char(usr[CONF_DATA]))
    
    # if CONF_CUSTOM in config:
    #     cg.add(var.set_custom_data(config[CONF_CUSTOM]))

    