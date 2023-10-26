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
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["water_quality"]

CONF_PUMP_TOTAL = "pump_total"
CONF_PUMP_STATUS = "pump_status"
CONF_LEVEL = "level"

UNIT_MILILITER = "ml"
UNIT_MILILITERS_PER_MINUTE = "ml/min"


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
            cv.Optional(CONF_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=2,
            )
            ,
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    # if CONF_PUMP_TOTAL in config:
    #     conf = config[CONF_PUMP_TOTAL]
    #     sens = await sensor.new_sensor(conf)
    #     cg.add(parent.PPump_0_Total(sens))
    #     cg.add(parent.PPump_1_Total(sens))
    #     cg.add(parent.PPump_2_Total(sens))
    #     cg.add(parent.PPump_3_Total(sens))
        
    # if CONF_PUMP_STATUS in config:
    #     conf = config[CONF_PUMP_STATUS]
    #     sens = await sensor.new_sensor(conf)
    #     cg.add(parent.PPump_0_Status(sens))
        
    if CONF_LEVEL in config:
        conf = config[CONF_LEVEL]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.AnLIn_Perc(sens))