import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.const import (
    CONF_ID,
    CONF_DATA
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
CONF_X = "x"
CONF_Y = "y"
CONF_PUMP_TYPE = "type"
CONF_PUMP_CALIBRATION = "pump_calibration"
CONF_PUMP_MODE = "pump_mode"
CONF_PUMP_DOSE = "pump_dose"

PUMP_TYPE_NULL = 0
PUMP_TYPE_DOSE = 1
PUMP_TYPE_CIRCULATION = 2
# PUMP_TYPES_SUPPORTED = [PUMP_TYPE_NULL, PUMP_TYPE_DOSE, PUMP_TYPE_CIRCULATION]


PUMP_CALIBRATION_SCHEMA = cv.Schema(
    {
        # cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_CALIBRATION): cv.All(
            cv.ensure_list(
                cv.Schema(
                    {
                        cv.Required(CONF_X): cv.All(
                            cv.ensure_list(cv.uint8_t),
                            cv.Length(min=8, max=8),
                        ),
                        cv.Required(CONF_Y): cv.All(
                            cv.ensure_list(cv.uint8_t),
                            cv.Length(min=8, max=8),
                        ),
                    }
                )
            )
        ),
    }
)
                    
PUMP_TYPE_SCHEMA = cv.typed_schema(
    {
        PUMP_TYPE_NULL: cv.Any({}),
        PUMP_TYPE_DOSE: PUMP_CALIBRATION_SCHEMA,
        PUMP_TYPE_CIRCULATION: PUMP_CALIBRATION_SCHEMA,
    },
    key=CONF_PUMP_TYPE,
    default_type=PUMP_TYPE_NULL,
    int=True,
)

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


PumpModeAction = component_ns.class_("PumpModeAction", automation.Action)
PumpDoseAction = component_ns.class_("PumpDoseAction", automation.Action)


PUMP_MODE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_MODE): (
            cv.All(
                # [cv.Any(cv.uint8_t)],
                # [cv.ensure_list(cv.uint8_t)],
                cv.templatable(cv.uint8_t),
                # cv.Length(min=0, max=3),
            )
        ),
        # cv.Required(CONF_PUMP_MODE):
        #     cv.templatable(cv.int_range()),
    }
)

PUMP_DOSE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        # cv.Required(CONF_DOSE): cv.templatable(
        #     cv.int_range()
        # ),
        cv.Required(CONF_PUMP_DOSE): (
            cv.All(
                [cv.Any(cv.uint8_t)],
            )
        ),
    }
)


@automation.register_action(
    "water_quality.pump_mode", 
    PumpModeAction, 
    PUMP_MODE_ACTION_SCHEMA
)

@automation.register_action(
    "water_quality.pump_dose", 
    PumpDoseAction, 
    PUMP_DOSE_ACTION_SCHEMA
)


async def pump_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    mode = config[CONF_PUMP_MODE]
    # template_ = await cg.templatable(mode, args, cg.uint8)
    template_ = await cg.templatable(mode, args, cg.std_vector.template(cg.uint8))
    cg.add(var.set_mode(template_))
    # cg.add(var.set_mode(mode))

    return var

async def pump_dose_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    # val = config[CONF_PUMP_DOSE]
    # # template_ = await cg.templatable(dose, args, cg.uint8)
    # # template_ = await cg.templatable(dose, args, cg.std_vector.template(cg.uint8))
    # # cg.add(var.set_dose(template_))
    # cg.add(var.set_dose(val))

    return var
