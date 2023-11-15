import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID, 
    CONF_GAIN, 
    CONF_POWER_SAVE_MODE,
    CONF_WHITE,
    UNIT_LUX,
)

from . import (
    component_ns, 
    VEML7700,
    CONF_COMP_ID
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["veml7700"]

CONF_INTETGRATION = "intetgration"
CONF_PERS = "persisance"
CONF_LUX = "lux"
CONF_ALS = "als"

UNIT_RAW = "raw"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_COMP_ID): cv.use_id(VEML7700),
            cv.Optional(CONF_LUX): sensor.sensor_schema(
                unit_of_measurement = UNIT_LUX,
                accuracy_decimals = 1,
            ),
            cv.Optional(CONF_WHITE): sensor.sensor_schema(
                unit_of_measurement = UNIT_RAW,
                accuracy_decimals = 1,
            ),
            cv.Optional(CONF_ALS): sensor.sensor_schema(
                unit_of_measurement = UNIT_RAW,
                accuracy_decimals = 1,
            ),
        }
    )
    # .extend(cv.COMPONENT_SCHEMA)
    # .extend(cv.polling_component_schema("60s"))
    # .extend(i2c.i2c_device_schema(0x76))
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_COMP_ID])

    if CONF_LUX in config:
        conf = config[CONF_LUX]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.Lux(sens))
        
    if CONF_WHITE in config:
        conf = config[CONF_WHITE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.White(sens))
        
    if CONF_ALS in config:
        conf = config[CONF_ALS]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.Als(sens))
        
        