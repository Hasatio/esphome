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
            cv.Optional(CONF_WATER_TEMP): sensor.sensor_schema(
                unit_of_measurement = UNIT_CELSIUS,
                accuracy_decimals = 2,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement = UNIT_VOLT,
                accuracy_decimals = 2,
            ),
            cv.Optional(CONF_EC): sensor.sensor_schema(
                unit_of_measurement = UNIT_MICROSIEMENS_PER_CENTIMETER,
                accuracy_decimals = 0,
            ),
            cv.Optional(CONF_PH): sensor.sensor_schema(
                unit_of_measurement = UNIT_PH,
                accuracy_decimals = 0,
            ),
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_WATER_TEMP in config:
        conf = config[CONF_WATER_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.WaterTemp_Sensor_Driver(sens))
        
    if CONF_VOLTAGE in config:
        conf = config[CONF_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.VPow_Sensor_Driver(sens))
        
    if CONF_EC in config:
        conf = config[CONF_EC]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.WaterEC_Sensor_Driver(sens))
        
    if CONF_PH in config:
        conf = config[CONF_PH]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.WaterPH_Sensor_Driver(sens))
        