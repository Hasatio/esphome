import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, 
    CONF_DATA,
    CONF_CUSTOM,
    CONF_CALIBRATION,
) 

AUTO_LOAD = ["sensor"]
MULTI_CONF = True

CONF_X = "x"
CONF_Y = "y"
CONF_PUMP_TOTAL = "pump_total"
CONF_PUMP_STATUS = "pump_status"
CONF_ANALOG_OUTPUT = "analog_output"

UNIT_MILILITER = "ml"
UNIT_MILILITERS_PER_MINUTE = "ml/min"

component_ns = cg.esphome_ns.namespace("water_quality")
MyComponent = component_ns.class_("MyComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MyComponent),
    cv.Optional(CONF_CALIBRATION): cv.All(
        cv.ensure_list(
            cv.Schema(
                {
                    cv.Required(CONF_X): cv.All(
                        cv.ensure_list(cv.uint8_t),
                        cv.Length(min=8, max=8),
                    ),
                    # cv.Required(CONF_Y): cv.All(
                    #     cv.ensure_list(cv.uint8_t),
                    #     cv.Length(min=8, max=8),
                    # ),
                }
            ).extend(cv.COMPONENT_SCHEMA),
        ),
        cv.Length(max=2),
    ),
    # cv.Optional(CONF_X): cv.ensure_list(cv.uint8_t),
    # cv.Optional(CONF_Y): cv.ensure_list(cv.uint8_t),
}).extend(cv.COMPONENT_SCHEMA)

# def to_code(config):
#     var = cg.new_Pvariable(config[CONF_ID])
#     yield cg.register_component(var, config)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_CALIBRATION in config:
        for usr in config[CONF_CALIBRATION]:
            cg.add(var.calibration(usr[CONF_X]))
    
    # if CONF_CALIBRATION in config:
    #     cg.add(var.calibration(config[CONF_X]))

    