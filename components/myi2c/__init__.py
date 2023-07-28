from typing import Optional

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, output
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
)

#DEPENDENCIES = ["i2c"]
CONF_MY_GAIN = "gain"
CONF_MY_BLUETOOTH = "bluetooth"
CONF_MY_SAMPLE = "sample"

UNIT_SAMPLE = "data/sec"

i2c_ns = cg.esphome_ns.namespace("myi2c") # esphome component adı
Myi2c = i2c_ns.class_("Myi2c", cg.Component)

CONFIG_SCHEMA = (
    cv.Schema(
        {
        cv.GenerateID(): cv.declare_id(Myi2c),
        cv.Optional(CONF_MY_GAIN): cv.float_,
        cv.Optional(CONF_MY_BLUETOOTH): cv.string,
        cv.Optional(CONF_MY_SAMPLE): sensor.sensor_schema(
                unit_of_measurement=UNIT_SAMPLE,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_MY_GAIN in config:
        cg.add(var.gain(config[CONF_MY_GAIN]))
    
    if CONF_MY_BLUETOOTH in config:
        cg.add(var.bluetooth(config[CONF_MY_BLUETOOTH]))

    if CONF_MY_SAMPLE in config:
        conf = config[CONF_MY_SAMPLE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.sample(sens))
