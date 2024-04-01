import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID, 
    CONF_ACTUAL_GAIN,
    CONF_AUTO_MODE,
    CONF_FULL_SPECTRUM,
    CONF_GAIN,
    CONF_GLASS_ATTENUATION_FACTOR,
    CONF_INFRARED,
    CONF_INTEGRATION_TIME,
    CONF_NAME,
    CONF_POWER_SAVE_MODE,
    CONF_WHITE,
    UNIT_LUX,
    UNIT_MILLISECOND,
)

from . import (
    component_ns, 
    VEML7700,
    CONF_COMP_ID
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["veml7700"]

CONF_AMBIENT_LIGHT = "ambient_light"
CONF_AMBIENT_LIGHT_COUNTS = "ambient_light_counts"
CONF_FULL_SPECTRUM_COUNTS = "full_spectrum_counts"
CONF_ACTUAL_INTEGRATION_TIME = "actual_integration_time"
CONF_LUX_COMPENSATION = "lux_compensation"

UNIT_RAW = "raw"
UNIT_COUNTS = "#"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_COMP_ID): cv.use_id(VEML7700),
            cv.Optional(CONF_AMBIENT_LIGHT): sensor.sensor_schema(
                unit_of_measurement = UNIT_LUX,
                accuracy_decimals = 1,
            ),
            cv.Optional(CONF_AMBIENT_LIGHT_COUNTS): sensor.sensor_schema(
                unit_of_measurement = UNIT_COUNTS,
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_FULL_SPECTRUM): sensor.sensor_schema(
                unit_of_measurement = UNIT_LUX,
                accuracy_decimals = 1,
            ),
            cv.Optional(CONF_FULL_SPECTRUM_COUNTS): sensor.sensor_schema(
                unit_of_measurement = UNIT_COUNTS,
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_INFRARED): sensor.sensor_schema(
                unit_of_measurement = UNIT_LUX,
                accuracy_decimals = 1,
            ),
            cv.Optional(CONF_ACTUAL_GAIN): sensor.sensor_schema(
                accuracy_decimals = 3,
            ),
            cv.Optional(CONF_ACTUAL_INTEGRATION_TIME): sensor.sensor_schema(
                unit_of_measurement = UNIT_MILLISECOND,
                accuracy_decimals = 0,
            ),
        }
    )
    # .extend(cv.COMPONENT_SCHEMA)
    # .extend(cv.polling_component_schema("60s"))
    # .extend(i2c.i2c_device_schema(0x76))
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_COMP_ID])

    if CONF_AMBIENT_LIGHT in config:
        conf = config[CONF_AMBIENT_LIGHT]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_ambient_light_sensor(sens))
        
    if CONF_AMBIENT_LIGHT_COUNTS in config:
        conf = config[CONF_AMBIENT_LIGHT_COUNTS]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_ambient_light_counts_sensor(sens))
        
    if CONF_FULL_SPECTRUM in config:
        conf = config[CONF_FULL_SPECTRUM]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_white_sensor(sens))
        
    if CONF_FULL_SPECTRUM_COUNTS in config:
        conf = config[CONF_FULL_SPECTRUM_COUNTS]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_white_counts_sensor(sens))
        
    if CONF_INFRARED in config:
        conf = config[CONF_INFRARED]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_infrared_sensor(sens))
        
    if CONF_ACTUAL_GAIN in config:
        conf = config[CONF_ACTUAL_GAIN]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_actual_gain_sensor(sens))
        
    if CONF_ACTUAL_INTEGRATION_TIME in config:
        conf = config[CONF_ACTUAL_INTEGRATION_TIME]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.set_actual_integration_time_sensor(sens))
        
        