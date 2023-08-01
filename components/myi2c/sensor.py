import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_NONE,
)

from . import Myi2c, CONF_MY_SAMPLE, UNIT_SAMPLE 

DEPENDENCIES = ["myi2c"]

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Myi2c),
            cv.Optional(CONF_MY_SAMPLE): sensor.sensor_schema(
                unit_of_measurement=UNIT_SAMPLE,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_EMPTY,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_NONE,
            ),
        }
    )
)

def to_code(config):
    parent = yield cg.get_variable(config[CONF_ID])

    if CONF_MY_SAMPLE in config:
        sens = yield sensor.new_sensor(config[CONF_MY_SAMPLE])
        cg.add(parent.sample(sens))


