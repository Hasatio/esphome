import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID, 
    CONF_STATUS, 
    CONF_TOTAL,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_PERCENT,
    UNIT_SECOND,
    UNIT_MILLISIEMENS_PER_CENTIMETER,
    UNIT_PH,
)

from . import (
    component_ns, 
    MyComponent,
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["water_quality"]

CONF_TIME_CONSTANT = "time_constant"
CONF_PUMP_TOTAL_ML = "pump_total_ml"
CONF_PUMP_TOTAL_L = "pump_total_l"
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

# MyComponent = component_ns.class_("MyComponent", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(MyComponent),
            cv.Optional(CONF_PUMP_TOTAL_ML): sensor.sensor_schema(
                unit_of_measurement = UNIT_MILILITER,
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_PUMP_TOTAL_L): sensor.sensor_schema(
                unit_of_measurement = UNIT_LITER,
                accuracy_decimals = 2,
            )
            # .extend(cv.polling_component_schema("1ms"))
            ,
            cv.Optional(CONF_PUMP_STATUS): sensor.sensor_schema(
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_SERVO_STATUS): sensor.sensor_schema(
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_WATER_TEMP): sensor.sensor_schema(
                unit_of_measurement = UNIT_CELSIUS,
                accuracy_decimals = 2,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement = UNIT_VOLT,
                accuracy_decimals = 2,
            ),
            cv.Optional(CONF_LEVEL): sensor.sensor_schema(
                unit_of_measurement = UNIT_PERCENT,
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_ANALOG): sensor.sensor_schema(
                unit_of_measurement = UNIT_VOLT,
                accuracy_decimals = 2,
            ),
            cv.Optional(CONF_EC): sensor.sensor_schema(
                unit_of_measurement = UNIT_MILLISIEMENS_PER_CENTIMETER,
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_PH): sensor.sensor_schema(
                unit_of_measurement = UNIT_PH,
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_DIGITAL): sensor.sensor_schema(
                accuracy_decimals = 0,
            ),
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_PUMP_TOTAL_ML in config:
        conf = config[CONF_PUMP_TOTAL_ML]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.PPump_Total_ml(sens))
        
    if CONF_PUMP_TOTAL_L in config:
        conf = config[CONF_PUMP_TOTAL_L]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.PPump_Total_l(sens))
        
    if CONF_PUMP_STATUS in config:
        conf = config[CONF_PUMP_STATUS]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.PPump_Status(sens))
        
    if CONF_SERVO_STATUS in config:
        conf = config[CONF_SERVO_STATUS]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.Servo_Status(sens))
        
    if CONF_WATER_TEMP in config:
        conf = config[CONF_WATER_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.AnInWT_Val(sens))
        
    if CONF_VOLTAGE in config:
        conf = config[CONF_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.AnInVPow_Val(sens))
        
    if CONF_LEVEL in config:
        conf = config[CONF_LEVEL]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.AnInLvl_Perc(sens))
        
    if CONF_ANALOG in config:
        conf = config[CONF_ANALOG]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.AnInGlob_Val(sens))
        
    if CONF_EC in config:
        conf = config[CONF_EC]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.AnInEC_Val(sens))
        
    if CONF_PH in config:
        conf = config[CONF_PH]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.AnInPH_Val(sens))
        
    if CONF_DIGITAL in config:
        conf = config[CONF_DIGITAL]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.DigIn_Status(sens))
        