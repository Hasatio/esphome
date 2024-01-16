import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID, 
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_PH,
)

from . import (
    component_ns, 
    WaterQuality,
    CONF_COMPONENT_ID
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["water_quality"]

CONF_WATER_TEMP = "water_temp"
CONF_VOLTAGE = "voltage"
CONF_EC = "ec"
CONF_PH = "ph"

UNIT_MICROSIEMENS_PER_CENTIMETER = "uS/cm"

MySensor = component_ns.class_("MySensor", sensor.Sensor, cg.PollingComponent)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_COMPONENT_ID): cv.use_id(WaterQuality),
            cv.Optional(CONF_WATER_TEMP): sensor.sensor_schema(
                # MySensor,
                unit_of_measurement = UNIT_CELSIUS,
                accuracy_decimals = 2,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                # MySensor,
                unit_of_measurement = UNIT_VOLT,
                accuracy_decimals = 2,
            ),
            cv.Optional(CONF_EC): sensor.sensor_schema(
                # MySensor,
                unit_of_measurement = UNIT_MICROSIEMENS_PER_CENTIMETER,
                accuracy_decimals = 1,
            ),
            cv.Optional(CONF_PH): sensor.sensor_schema(
                # MySensor,
                unit_of_measurement = UNIT_PH,
                accuracy_decimals = 1,
            ),
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_COMPONENT_ID])

    if CONF_WATER_TEMP in config:
        conf = config[CONF_WATER_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.WT_Val_Sensor(sens))
        
    if CONF_VOLTAGE in config:
        conf = config[CONF_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.VP_Val_Sensor(sens))
        
    if CONF_EC in config:
        conf = config[CONF_EC]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.EC_Val_Sensor(sens))
        
    if CONF_PH in config:
        conf = config[CONF_PH]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.PPH_Val_Sensor(sens))
        