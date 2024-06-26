import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome import automation
from esphome.const import (
    CONF_ID,
    CONF_DATA
) 

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "text_sensor"]
MULTI_CONF = True

CONF_WATER_QUALITY = "water_quality"
CONF_COMPONENT_ID = "component_id"
CONF_VERSION = "version"
CONF_PUMP1 = "pump1"
CONF_PUMP2 = "pump2"
CONF_PUMP3 = "pump3"
CONF_PUMP4 = "pump4"
CONF_PUMP5 = "pump5"
CONF_PUMP6 = "pump6"
CONF_PUMP_CALIBRATION_MODE = "pump_calibration_mode"
CONF_PUMP_CALIBRATION_GAIN = "pump_calibration_gain"
CONF_PUMP_TYPE = "pump_type"
CONF_PUMP_CIRCULATION_MODEL = "pump_circulation_model"
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
CONF_EC_CALIBRATION = "ec_calibration"
CONF_EC_CHANNEL = "ec_channel"
CONF_EC_TYPE = "ec_type"
CONF_PH_CALIBRATION = "ph_calibration"
CONF_PH_CHANNEL = "ph_channel"
CONF_PH_TYPE = "ph_type"
CONF_DIGITAL_OUT = "digital_out"
CONF_CUSTOM_COMMAND = "custom_command"

PUMP_TYPE_NULL = 0
PUMP_TYPE_DOSE = 1
PUMP_TYPE_CIRCULATION = 2


PUMP_DOSING_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PUMP_CALIBRATION_GAIN): cv.float_,
    }
)
PUMP_CIRCULATION_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PUMP_CALIBRATION_GAIN): cv.float_,
    }
)

PUMP_TYPE_SCHEMA = cv.typed_schema(
    {
        PUMP_TYPE_NULL: cv.Any({}),
        PUMP_TYPE_DOSE: PUMP_DOSING_SCHEMA,
        PUMP_TYPE_CIRCULATION: PUMP_CIRCULATION_SCHEMA,
    },
    key=CONF_PUMP_TYPE,
    default_type=PUMP_TYPE_NULL,
    int=True,
)

water_quality_ns = cg.esphome_ns.namespace(CONF_WATER_QUALITY)
WaterQuality = water_quality_ns.class_("WaterQuality", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WaterQuality),
            cv.Optional(CONF_VERSION, default = 1): cv.uint8_t,  
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
                ),
            ),
            cv.Required(CONF_SENSORS): cv.All(
                cv.ensure_list(
                    cv.Schema(
                        {
                            cv.Required(CONF_EC_CHANNEL): cv.All(
                                cv.int_range(min = 0, max = 4),
                            ),
                            cv.Required(CONF_EC_TYPE): cv.All(
                                cv.int_range(min = 0, max = 2),
                            ),
                            cv.Required(CONF_PH_CHANNEL): cv.All(
                                cv.int_range(min = 0, max = 4),
                            ),
                            cv.Required(CONF_PH_TYPE): cv.All(
                                cv.int_range(min = 0, max = 2),
                            ),
                        }
                    )
                ),
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(None)),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # parent = await cg.get_variable(config["WQ_I2C"])
    # cg.add(var.set_parent(parent))
    
    if CONF_VERSION in config:
        cg.add(var.version(config[CONF_VERSION]))
        
    zero = [0] * 1
    type = []
    calib = []
    
    conf = config[CONF_PUMP1][0]
    type.append(conf[CONF_PUMP_TYPE])
    if conf[CONF_PUMP_TYPE] != 0:
        calib.append(conf[CONF_PUMP_CALIBRATION_GAIN])
    else:
        calib.append(zero)
            
    conf = config[CONF_PUMP2][0]
    type.append(conf[CONF_PUMP_TYPE])
    if conf[CONF_PUMP_TYPE] != 0:
        calib.append(conf[CONF_PUMP_CALIBRATION_GAIN])
    else:
        calib.append(zero)
        
    conf = config[CONF_PUMP3][0]
    type.append(conf[CONF_PUMP_TYPE])
    if conf[CONF_PUMP_TYPE] != 0:
        calib.append(conf[CONF_PUMP_CALIBRATION_GAIN])
    else:
        calib.append(zero)
        
    conf = config[CONF_PUMP4][0]
    type.append(conf[CONF_PUMP_TYPE])
    if conf[CONF_PUMP_TYPE] != 0:
        calib.append(conf[CONF_PUMP_CALIBRATION_GAIN])
    else:
        calib.append(zero)
        
    conf = config[CONF_PUMP5][0]
    type.append(conf[CONF_PUMP_TYPE])
    if conf[CONF_PUMP_TYPE] != 0:
        calib.append(conf[CONF_PUMP_CALIBRATION_GAIN])
    else:
        calib.append(zero)
        
    conf = config[CONF_PUMP6][0]
    type.append(conf[CONF_PUMP_TYPE])
    if conf[CONF_PUMP_TYPE] != 0:
        calib.append(conf[CONF_PUMP_CALIBRATION_GAIN])
    else:
        calib.append(zero)
        
    cg.add(var.pump_calibration_gain(calib))
    cg.add(var.pump_type(type))
    
        
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

    cg.add_library("EEPROM", None)
    cg.add_library("DFRobot_EC", None, "https://github.com/DFRobot/DFRobot_EC/")
    cg.add_library("DFRobot_EC10", None)
    cg.add_library("DFRobot_PH", None)
    cg.add_library("SPI", None)
    cg.add_library("SD", None)
    cg.add_library("Time", None)
    
    
Pump_Calibration_Mode_Action = water_quality_ns.class_("Pump_Calibration_Mode_Action", automation.Action)

PUMP_CALIBRATION_MODE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_PUMP_CALIBRATION_MODE): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.pump_calibration_mode", 
    Pump_Calibration_Mode_Action, 
    PUMP_CALIBRATION_MODE_ACTION_SCHEMA
)

async def pump_calibration_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_CALIBRATION_MODE]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.bool_))
        cg.add(var.set_pump_calib_mode(template_))

    return var


Pump_Mode_Action = water_quality_ns.class_("Pump_Mode_Action", automation.Action)

PUMP_MODE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_PUMP_MODE): (
            cv.All(
                cv.templatable(
                    cv.ensure_list(cv.int_range(min = 0, max = 3)),
                    # cv.Length(min=0, max=3),
                ),
            )
        ),
    }
)

@automation.register_action(
    "water_quality.pump_mode", 
    Pump_Mode_Action, 
    PUMP_MODE_ACTION_SCHEMA
)

async def pump_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_MODE]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_pump_m(template_))

    return var


Pump_Dose_Action = water_quality_ns.class_("Pump_Dose_Action", automation.Action)

PUMP_DOSE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_PUMP_DOSE): cv.All(
            cv.templatable(
                cv.ensure_list(cv.float_)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.pump_dose", 
    Pump_Dose_Action, 
    PUMP_DOSE_ACTION_SCHEMA
)

async def pump_dose_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_DOSE]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.float_))
        cg.add(var.set_pump_d(template_))

    return var


Pump_Circulation_Action = water_quality_ns.class_("Pump_Circulation_Action", automation.Action)

PUMP_CIRCULATION_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_PUMP_CIRCULATION): cv.All(
            cv.templatable(
                cv.ensure_list(cv.float_)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.pump_circulation", 
    Pump_Circulation_Action, 
    PUMP_CIRCULATION_ACTION_SCHEMA
)

async def pump_circulation_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_CIRCULATION]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.float_))
        cg.add(var.set_pump_c(template_))

    return var


Pump_Reset_Action = water_quality_ns.class_("Pump_Reset_Action", automation.Action)

PUMP_RESET_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_PUMP_RESET): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.pump_reset", 
    Pump_Reset_Action, 
    PUMP_RESET_ACTION_SCHEMA
)

async def pump_reset_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PUMP_RESET]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.bool_))
        cg.add(var.set_pump_res(template_))

    return var


Servo_Mode_Action = water_quality_ns.class_("Servo_Mode_Action", automation.Action)

SERVO_MODE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_SERVO_MODE): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.servo_mode", 
    Servo_Mode_Action, 
    SERVO_MODE_ACTION_SCHEMA
)

async def servo_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_SERVO_MODE]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.bool_))
        cg.add(var.set_ser_mode(template_))

    return var


Servo_Position_Action = water_quality_ns.class_("Servo_Position_Action", automation.Action)

SERVO_POSITION_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_SERVO_POSITION): cv.All(
            cv.templatable(
                cv.ensure_list(cv.uint8_t)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.servo_position", 
    Servo_Position_Action, 
    SERVO_POSITION_ACTION_SCHEMA
)

async def servo_position_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_SERVO_POSITION]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_ser_pos(template_))

    return var


PH_Calibration_Action = water_quality_ns.class_("PH_Calibration_Action", automation.Action)

PH_CALIBRATION_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_PH_CALIBRATION): cv.All(
            cv.templatable(cv.float_range(min = 0, max = 14)),
        ),
    }
)

@automation.register_action(
    "water_quality.ph_calibration", 
    PH_Calibration_Action, 
    PH_CALIBRATION_ACTION_SCHEMA
)

async def ph_calibration_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_PH_CALIBRATION]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.float_)
        cg.add(var.set_ph_cal(template_))

    return var


EC_Calibration_Action = water_quality_ns.class_("EC_Calibration_Action", automation.Action)

EC_CALIBRATION_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_EC_CALIBRATION): cv.All(
            cv.templatable(cv.float_range(min = 0, max = 20)),
        ),
    }
)

@automation.register_action(
    "water_quality.ec_calibration", 
    EC_Calibration_Action, 
    EC_CALIBRATION_ACTION_SCHEMA
)

async def ec_calibration_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_EC_CALIBRATION]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.float_)
        cg.add(var.set_ec_cal(template_))

    return var


Digital_Out_Action = water_quality_ns.class_("Digital_Out_Action", automation.Action)

DIGITAL_OUT_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(WaterQuality),
        cv.Required(CONF_DIGITAL_OUT): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "water_quality.digital_out", 
    Digital_Out_Action, 
    DIGITAL_OUT_ACTION_SCHEMA
)

async def digital_out_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_DIGITAL_OUT]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.bool_))
        cg.add(var.set_dig_out(template_))

    return var

