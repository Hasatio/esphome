from typing import Optional

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, output
from esphome.const import (CONF_ID)

#DEPENDENCIES = ["i2c"]
CONF_MY_OUTPUT = "output"
CONF_MY_BLUETOOTH = "bluetooth"

i2c_ns = cg.esphome_ns.namespace("myi2c")
Myi2cComponent = i2c_ns.class_("Myi2cComponent", cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
        cv.GenerateID(): cv.declare_id(Custom),
        cv.Optional(CONF_MY_OUTPUT): cv.float_,
        cv.Optional(CONF_MY_BLUETOOTH): cv.std_string
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.gain(config[CONF_MY_OUTPUT]))
    cg.add(var.device(config[CONF_MY_BLUETOOTH]))
