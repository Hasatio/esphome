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

CONF_PUMP_TOTAL = "Pump_Total"
CONF_PUMP_STATUS = "Pump_Status"
CONF_SERVO_STATUS = "Servo_Status"
CONF_LEVEL_PERCENTAGE = "Level_Percentage"
CONF_ANALOG_INPUT = "Analog_Input"
CONF_DIGITAL_INPUT = "Digital_Input"

UNIT_LITER = "L"
UNIT_MILILITER = "mL"
UNIT_MILILITERS_PER_MINUTE = "mL/min"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_COMPONENT_ID): cv.use_id(WaterQuality),
            cv.Optional(CONF_PUMP_TOTAL): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_PUMP_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_SERVO_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LEVEL_PERCENTAGE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_ANALOG_INPUT): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_DIGITAL_INPUT): text_sensor.text_sensor_schema(),
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_COMPONENT_ID])

    if CONF_PUMP_TOTAL in config:
        conf = config[CONF_PUMP_TOTAL]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.PPump_Tot_Sensor(sens))
        
    if CONF_PUMP_STATUS in config:
        conf = config[CONF_PUMP_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.PPump_Stat_Sensor(sens))
        
    if CONF_SERVO_STATUS in config:
        conf = config[CONF_SERVO_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.Servo_Stat_Sensor(sens))
        
    if CONF_LEVEL_PERCENTAGE in config:
        conf = config[CONF_LEVEL_PERCENTAGE]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.AnLvl_Perc_Sensor(sens))
        
    if CONF_ANALOG_INPUT in config:
        conf = config[CONF_ANALOG_INPUT]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.AnGen_Val_Sensor(sens))
        
    if CONF_DIGITAL_INPUT in config:
        conf = config[CONF_DIGITAL_INPUT]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.DigIn_Stat_Sensor(sens))
        