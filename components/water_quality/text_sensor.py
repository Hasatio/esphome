import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, text_sensor
from esphome.const import (
    CONF_ID, 
    CONF_STATUS, 
    CONF_TOTAL,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_PERCENT,
    UNIT_SECOND,
    UNIT_PH,
)

from . import (
    component_ns, 
    MyComponent,
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["water_quality"]

CONF_PUMP_TOTAL = "pump_total"
CONF_PUMP_STATUS = "pump_status"
CONF_SERVO_STATUS = "servo_status"
CONF_WATER_TEMP = "water_temp"
CONF_VOLTAGE = "voltage"
CONF_LEVEL = "level"
CONF_ANALOG = "analog"
CONF_EC = "ec"
CONF_PH = "ph"
CONF_DIGITAL = "digital"

UNIT_LITER = "l"
UNIT_MILILITER = "ml"
UNIT_MILILITERS_PER_MINUTE = "ml/min"
UNIT_MICROSIEMENS_PER_CENTIMETER = "uS/cm"

# MyComponent = component_ns.class_("MyComponent", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(MyComponent),
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
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_PUMP_TOTAL in config:
        conf = config[CONF_PUMP_TOTAL]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.PPump_Tot(sens))
        
    if CONF_PUMP_STATUS in config:
        conf = config[CONF_PUMP_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.PPump_Stat(sens))
        
    if CONF_SERVO_STATUS in config:
        conf = config[CONF_SERVO_STATUS]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.Servo_Stat(sens))
        
    if CONF_LEVEL in config:
        conf = config[CONF_LEVEL]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.AnLevel_Sensor_Driver(sens))
        
    if CONF_ANALOG in config:
        conf = config[CONF_ANALOG]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.AnGen_Input_Driver(sens))
        
    if CONF_DIGITAL in config:
        conf = config[CONF_DIGITAL]
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.DigIn_Stat(sens))
        