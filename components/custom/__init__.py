from typing import Optional

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, output
from esphome.const import (
CONF_ID,
CONF_BAUD_RATE,
CONF_FLOAT,
)

#DEPENDENCIES = ["i2c"]

custom_ns = cg.esphome_ns.namespace("custom")
Custom = custom_ns.class_("Custom", output.FloatOutput, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Custom),
    cv.Optional(CONF_BAUD_RATE): cv.int_range(min=1),
    cv.Optional(CONF_FLOAT): cv.int_range(min=0.0)
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_float(config[CONF_FLOAT]))
