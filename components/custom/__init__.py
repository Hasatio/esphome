from typing import Optional

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, output
from esphome.const import (
CONF_ID,
CONF_VARIABLES
)
from esphome import automation
from esphome.automation import maybe_simple_id

#DEPENDENCIES = ["i2c"]
CONF_ON_CUSTOM = "on_custom"

custom_ns = cg.esphome_ns.namespace("custom")
Custom = custom_ns.class_("Custom", output.FloatOutput, cg.Component)
Custom_action = custom_ns.class_("Custom_action", automation.Action)

CONFIG_SCHEMA = (cv.Schema({
    cv.GenerateID(): cv.declare_id(Custom),
}).extend(cv.COMPONENT_SCHEMA)
)

CUSTOM_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(Custom),
        cv.Optional(CONF_ON_CUSTOM): cv.templatable(cv.float_range()),
    }
)

@automation.register_action(
    "custom.set_variables",
    Custom_action,
    CUSTOM_ACTION_SCHEMA,)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield output.register_output(var, config)
    cg.add(var.set_variables(config[CONF_ON_CUSTOM]))
    
