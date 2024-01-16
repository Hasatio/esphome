import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, text_sensor
from esphome.const import (
    CONF_ID, 
)

from . import (
    component_ns, 
    WaterQuality,
    CONF_COMPONENT_ID
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["water_quality"]

CONF_PUMP_TOTAL = "pump_total"
CONF_PUMP_STATUS = "pump_status"
CONF_SERVO_STATUS = "servo_status"
CONF_LEVEL = "level"
CONF_ANALOG = "analog"
CONF_DIGITAL = "digital"

UNIT_LITER = "l"
UNIT_MILILITER = "ml"
UNIT_MILILITERS_PER_MINUTE = "ml/min"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_COMPONENT_ID): cv.use_id(WaterQuality),
            cv.Optional(CONF_PUMP_TOTAL): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_PUMP_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_SERVO_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LEVEL): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_ANALOG): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_DIGITAL): text_sensor.text_sensor_schema(),
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_COMPONENT_ID])

    if CONF_PUMP_TOTAL in config:
        conf = config[CONF_PUMP_TOTAL]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.Pump_Tot_Sensor(sens))
        
    if CONF_PUMP_STATUS in config:
        conf = config[CONF_PUMP_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.Pump_Stat_Sensor(sens))
        
    if CONF_SERVO_STATUS in config:
        conf = config[CONF_SERVO_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.Servo_Stat_Sensor(sens))
        
    if CONF_LEVEL in config:
        conf = config[CONF_LEVEL]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.AnLvl_Perc_Sensor(sens))
        
    if CONF_ANALOG in config:
        conf = config[CONF_ANALOG]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.AnGen_Val_Sensor(sens))
        
    if CONF_DIGITAL in config:
        conf = config[CONF_DIGITAL]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.DigIn_Stat_Sensor(sens))
        