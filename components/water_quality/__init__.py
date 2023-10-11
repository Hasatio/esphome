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

CONF_X1 = "x1"
CONF_Y1 = "y1"
CONF_X2 = "x2"
CONF_Y2 = "y2"
CONF_X3 = "x3"
CONF_Y3 = "y3"
CONF_X4 = "x4"
CONF_Y4 = "y4"
CONF_PUMP_TOTAL = "pump_total"
CONF_PUMP_STATUS = "pump_status"
CONF_ANALOG_OUTPUT = "analog_output"

UNIT_MILILITER = "ml"
UNIT_MILILITERS_PER_MINUTE = "ml/min"

component_ns = cg.esphome_ns.namespace("water_quality")
MyComponent = component_ns.class_("MyComponent", cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MyComponent),
            cv.Optional(CONF_CALIBRATION): cv.All(
                cv.ensure_list(
                    cv.Schema(
                        {
                            cv.Required(CONF_X1): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y1): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_X2): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y2): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_X3): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y3): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_X4): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y4): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                        }
                    ).extend(cv.COMPONENT_SCHEMA),
                ),
                cv.Length(max=8),
            ),
            # cv.Optional(CONF_X): cv.ensure_list(cv.uint8_t),
            # cv.Optional(CONF_Y): cv.ensure_list(cv.uint8_t),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_CALIBRATION in config:
        for conf in config[CONF_CALIBRATION]:
            cg.add(var.calibration(
                conf[CONF_X1], 
                conf[CONF_Y1],
                conf[CONF_X2], 
                conf[CONF_Y2],
                conf[CONF_X3], 
                conf[CONF_Y3],
                conf[CONF_X4], 
                conf[CONF_Y4],
                ))
    