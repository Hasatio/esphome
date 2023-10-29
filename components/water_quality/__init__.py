import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.const import (
    CONF_ID,
    CONF_DATA
) 

CODEOWNERS = ["@hasatio"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = True

CONF_PUMP1 = "pump1"
CONF_PUMP2 = "pump2"
CONF_PUMP3 = "pump3"
CONF_PUMP4 = "pump4"
CONF_PUMP5 = "pump5"
CONF_PUMP6 = "pump6"
CONF_X = "x"
CONF_Y = "y"
CONF_PUMP_TYPE = "pump_type"
CONF_PUMP_CALIBRATION = "pump_calibration"
CONF_PUMP_MODE = "pump_mode"
CONF_PUMP_DOSE = "pump_dose"
CONF_PUMP_CIRCULATION = "pump_circulation"
CONF_PUMP_RESET = "pump_reset"
CONF_SERVO_MODE = "servo_mode"
CONF_SERVO_POSITION = "servo_position"
CONF_LEVEL = "level"
CONF_RES_MIN = "res_min"
CONF_RES_MAX = "res_max"
CONF_SENSORS = "sensors"
CONF_EC_CHANNEL = "ec_channel"
CONF_EC_TYPE = "ec_type"
CONF_PH_CHANNEL = "ph_channel"
CONF_PH_TYPE = "ph_type"
CONF_DIGITAL_OUT = "digital_out"

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
                            cv.Length(min = 8, max = 8),
                        ),
                        cv.Required(CONF_Y): cv.All(
                            cv.ensure_list(cv.uint8_t),
                            cv.Length(min = 8, max = 8),
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
                cv.ensure_list(PUMP_TYPE_SCHEMA),
                cv.Length(min = 1)
            ),
            cv.Required(CONF_PUMP2): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA),
                cv.Length(min = 1)
            ),
            cv.Required(CONF_PUMP3): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA),
                cv.Length(min = 1)
            ),
            cv.Required(CONF_PUMP4): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA),
                cv.Length(min = 1)
            ),
            cv.Required(CONF_PUMP5): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA),
                cv.Length(min = 1)
            ),
            cv.Required(CONF_PUMP6): cv.All(
                cv.ensure_list(PUMP_TYPE_SCHEMA),
                cv.Length(min = 1)
            ),
            cv.Required(CONF_LEVEL): cv.All(
                cv.ensure_list(
                    cv.Schema(
                        {
                            cv.Required(CONF_RES_MIN): cv.All(
                                cv.ensure_list(cv.uint16_t),
                                cv.Length(min = 1, max = 2)
                            ),
                            cv.Required(CONF_RES_MAX): cv.All(
                                cv.ensure_list(cv.uint16_t),
                                cv.Length(min = 1, max = 2)
                            ),  
                        }
                    )
                )
            ),
            cv.Required(CONF_SENSORS): cv.All(
                cv.ensure_list(
                    cv.Schema(
                        {
                            cv.Required(CONF_EC_CHANNEL): cv.All(
                                cv.int_range(min = 0, max = 4),
                                # cv.Length(min = 1, max = 1)
                            ),
                            cv.Required(CONF_EC_TYPE): cv.All(
                                cv.int_range(min = 0, max = 2),
                                # cv.Length(min = 1, max = 1)
                            ),
                            cv.Required(CONF_PH_CHANNEL): cv.All(
                                cv.int_range(min = 0, max = 4),
                                # cv.Length(min = 1, max = 1)
                            ),
                            cv.Required(CONF_PH_TYPE): cv.All(
                                cv.int_range(min = 0, max = 2),
                                # cv.Length(min = 1, max = 1)
                            ),
                        }
                    )
                )
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
    dose = 0
    circ = 0
    
    con = config[CONF_PUMP1][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        if con[CONF_PUMP_TYPE] == 1:
            dose += 1
        else:
            circ += 1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
            
    con = config[CONF_PUMP2][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        if con[CONF_PUMP_TYPE] == 1:
            dose += 1
        else:
            circ += 1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP3][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        if con[CONF_PUMP_TYPE] == 1:
            dose += 1
        else:
            circ += 1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP4][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        if con[CONF_PUMP_TYPE] == 1:
            dose += 1
        else:
            circ += 1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP5][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        if con[CONF_PUMP_TYPE] == 1:
            dose += 1
        else:
            circ += 1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    con = config[CONF_PUMP6][0]
    type.append(con[CONF_PUMP_TYPE])
    if con[CONF_PUMP_TYPE] != 0:
        if con[CONF_PUMP_TYPE] == 1:
            dose += 1
        else:
            circ += 1
        for conf in con[CONF_PUMP_CALIBRATION]:
            calib.append(conf[CONF_X])
            calib.append(conf[CONF_Y])
        
    cg.add(var.pump_calibration(calib))
    cg.add(var.pump_type(type, dose, circ))
    
        
    min = []
    max = []
    for conf in config[CONF_LEVEL]:
        min.append(conf[CONF_RES_MIN])
        max.append(conf[CONF_RES_MAX])
    cg.add(var.level_res(min, max))
    
    conf = config[CONF_SENSORS][0]
    ch = conf[CONF_EC_CHANNEL]
    t = conf[CONF_EC_TYPE]
    cg.add(var.ec(ch, t))
    ch = conf[CONF_PH_CHANNEL]
    t = conf[CONF_PH_TYPE]
    cg.add(var.ph(ch, t))

    cg.add_library("Wire", None)
    cg.add_library("SPI", None)
    cg.add_library("Adafruit BusIO",None)
    cg.add_library("Adafruit ADS1X15", None)
    cg.add_library("Adafruit MCP23017 Arduino Library", None)
    cg.add_library("Adafruit PWM Servo Driver Library", None)


PumpModeAction = component_ns.class_("PumpModeAction", automation.Action)

PUMP_MODE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_MODE): (
            cv.All(
                # [cv.Any(cv.uint8_t)],
                # [cv.ensure_list(cv.uint8_t)],
                cv.templatable(
                    cv.ensure_list(cv.int_range(min = 0, max = 2))
                ),
                # cv.Length(min=0, max=3),
            )
        ),
        # cv.Required(CONF_PUMP_MODE):
        #     cv.templatable(cv.int_range()),
    }
)

@automation.register_action(
    "water_quality.pump_mode", 
    PumpModeAction, 
    PUMP_MODE_ACTION_SCHEMA
)

async def pump_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_MODE]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_val(template_))

    return var


PumpDoseAction = component_ns.class_("PumpDoseAction", automation.Action)

PUMP_DOSE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_DOSE): cv.All(
            cv.templatable(
                cv.ensure_list(cv.uint8_t)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.pump_dose", 
    PumpDoseAction, 
    PUMP_DOSE_ACTION_SCHEMA
)

async def pump_dose_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_DOSE]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_val(template_))

    return var


PumpCirculationAction = component_ns.class_("PumpCirculationAction", automation.Action)

PUMP_CIRCULATION_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_CIRCULATION): cv.All(
            cv.templatable(
                cv.ensure_list(cv.uint16_t)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.pump_circulation", 
    PumpCirculationAction, 
    PUMP_CIRCULATION_ACTION_SCHEMA
)

async def pump_circulation_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_CIRCULATION]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.uint16))
        cg.add(var.set_val(template_))

    return var


PumpResetAction = component_ns.class_("PumpResetAction", automation.Action)

PUMP_RESET_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_RESET): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.pump_reset", 
    PumpResetAction, 
    PUMP_RESET_ACTION_SCHEMA
)

async def pump_reset_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_RESET]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.bool_))
        cg.add(var.set_val(template_))

    return var


ServoModeAction = component_ns.class_("ServoModeAction", automation.Action)

SERVO_MODE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_SERVO_MODE): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.servo_mode", 
    ServoModeAction, 
    SERVO_MODE_ACTION_SCHEMA
)

async def servo_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_SERVO_MODE]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.bool_))
        cg.add(var.set_val(template_))

    return var


ServoPositionAction = component_ns.class_("ServoModeAction", automation.Action)

SERVO_POSITION_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_SERVO_POSITION): cv.All(
            cv.templatable(
                cv.ensure_list(cv.uint8_t)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.servo_position", 
    ServoPositionAction, 
    SERVO_POSITION_ACTION_SCHEMA
)

async def servo_position_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_SERVO_POSITION]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_val(template_))

    return var


DigitalOutAction = component_ns.class_("DigitalOutAction", automation.Action)

DIGITAL_OUT_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_DIGITAL_OUT): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.digital_out", 
    DigitalOutAction, 
    DIGITAL_OUT_ACTION_SCHEMA
)

async def digital_out_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_DIGITAL_OUT]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.bool_))
        cg.add(var.set_val(template_))

    return var
