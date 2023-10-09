import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, 
    CONF_STATUS, 
    CONF_TOTAL,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_PERCENT,
)

from . import (
    component_ns, 
    MyComponent, 
    CONF_PUMP_TOTAL,
    CONF_PUMP_STATUS,
    CONF_ANALOG_OUTPUT,
    UNIT_MILILITER, 
    UNIT_MILILITERS_PER_MINUTE,
)

DEPENDENCIES = ["water_quality"]

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(MyComponent),
            cv.Optional(CONF_PUMP_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILILITER,
                accuracy_decimals=2,
            )
            # .extend(cv.polling_component_schema("1ms"))
            ,
            cv.Optional(CONF_PUMP_STATUS): sensor.sensor_schema(
                # accuracy_decimals=0,
            )
            ,
            # cv.Optional(CONF_ANALOG_OUTPUT): sensor.sensor_schema(
            #     unit_of_measurement=UNIT_CELSIUS,
            #     accuracy_decimals=1,
            # )
            # ,
            # cv.Optional(CONF_ANALOG_OUTPUT): sensor.sensor_schema(
            #     unit_of_measurement=UNIT_VOLT,
            #     accuracy_decimals=2,
            # )
            # ,
            cv.Optional(CONF_ANALOG_OUTPUT): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=2,
            )
            ,
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_PUMP_TOTAL in config:
        sens = await sensor.new_sensor(config[CONF_PUMP_TOTAL])
        cg.add(parent.PPump_0_Total(sens))
        # cg.add(parent.Pump_1_Total(sens))
        # cg.add(parent.Pump_2_Total(sens))
        # cg.add(parent.Pump_3_Total(sens))
        
    if CONF_PUMP_STATUS in config:
        sens = await sensor.new_sensor(config[CONF_PUMP_STATUS])
        cg.add(parent.Pump_0_Status(sens))
        
    if CONF_ANALOG_OUTPUT in config:
        sens = await sensor.new_sensor(config[CONF_ANALOG_OUTPUT])
        cg.add(parent.AnOut_2_LvlPerc(sens))
        cg.add(parent.AnOut_3_LvlPerc(sens))