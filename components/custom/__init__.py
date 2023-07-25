from typing import Optional

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, output
from esphome.const import (
CONF_ID,
CONF_VARIABLES
)

#DEPENDENCIES = ["i2c"]
CONF_ON_CUSTOM = "on_custom"

custom_ns = cg.esphome_ns.namespace("custom")
Custom = custom_ns.class_("Custom", output.FloatOutput, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Custom),
    cv.Optional(CONF_ON_CUSTOM): cv.float_
}).extend(cv.COMPONENT_SCHEMA)

@automation.register_action(
    "custom.set_variables", 
    CONFIG_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield output.register_output(var, config)
    cg.add(var.set_variables(config[CONF_ON_CUSTOM]))
    
