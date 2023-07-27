from typing import Optional

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, output
from esphome.const import (CONF_ID)

#DEPENDENCIES = ["i2c"]
CONF_MY_BLUETOOTH = "mybluetooth"
CONF_MY_OUTPUT = "myoutput"

i2c_ns = cg.esphome_ns.namespace("myi2c")
Myi2c = i2c_ns.class_("Myi2c", output.FloatOutput, cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
        cv.GenerateID(): cv.declare_id(Myi2c),
        cv.Optional(CONF_MY_OUTPUT): cv.float_range(),
        cv.Optional(CONF_MY_BLUETOOTH): cv.string()
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    # yield output.register_output(var, config)
    yield cg.register_component(var, config)
    cg.add(var.gain(config[CONF_MY_OUTPUT]))
    cg.add(var.device(config[CONF_MY_BLUETOOTH]))
