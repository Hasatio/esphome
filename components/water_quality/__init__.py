import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_DATA
) 

from . import (
    CONF_X,
    CONF_Y,
    CONF_PUMP_TYPE,
    CONF_PUMP_CALIBRATION, 
    PUMP_TYPE_SCHEMA,
)

CODEOWNERS = ["@hasatio"]
AUTO_LOAD = ["sensor", "output"]
MULTI_CONF = True

CONF_PUMP1 = "pump1"
CONF_PUMP2 = "pump2"
CONF_PUMP3 = "pump3"
CONF_PUMP4 = "pump4"
CONF_PUMP5 = "pump5"
CONF_PUMP6 = "pump6"


component_ns = cg.esphome_ns.namespace("water_quality")
MyComponent = component_ns.class_("MyComponent", cg.Component)

                    
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MyComponent),
            cv.Required(CONF_PUMP1): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA), cv.Length(min=1)
            ),
            cv.Required(CONF_PUMP2): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA), cv.Length(min=1)
            ),
            cv.Required(CONF_PUMP3): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA), cv.Length(min=1)
            ),
            cv.Required(CONF_PUMP4): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA), cv.Length(min=1)
            ),
            cv.Required(CONF_PUMP5): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA), cv.Length(min=1)
            ),
            cv.Required(CONF_PUMP6): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA), cv.Length(min=1)
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    type = []
    calib = []
    num = 0
    
    con = config[CONF_PUMP1][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        num +=1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
            
    con = config[CONF_PUMP2][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        num +=1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP3][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        num +=1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP4][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        num +=1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP5][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        num +=1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP6][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        num +=1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    cg.add(var.pump_calibration(calib))
    cg.add(var.pump_type(type, num))
            