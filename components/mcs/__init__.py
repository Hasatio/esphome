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

CONF_COMPONENT_ID = "component_id"
CONF_VERSION = "version"
CONF_DIGITAL_OUT = "digital_out"
CONF_DIGITAL_OUT2 = "digital_out2"
CONF_CUSTOM_COMMAND = "custom_command"


mcs_ns = cg.esphome_ns.namespace("mcs")
MCS = mcs_ns.class_("MCS", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MCS),
            cv.Optional(CONF_VERSION, default = 1): cv.uint8_t,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(None)),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    
    if CONF_VERSION in config:
        cg.add(var.version(config[CONF_VERSION]))
    

    cg.add_library("EEPROM", None)
    cg.add_library("SPI", None)
    cg.add_library("SD", None)
    cg.add_library("Time", None)
    

Digital_Out_Action = mcs_ns.class_("Digital_Out_Action", automation.Action)

DIGITAL_OUT_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MCS),
        cv.Required(CONF_DIGITAL_OUT): cv.All(
            cv.templatable(
                cv.ensure_list(cv.boolean)
            ),
        ),
    }
)

@automation.register_action(
    "mcs.digital_out", 
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


Digital_Out_Action2 = mcs_ns.class_("Digital_Out_Action2", automation.Action)

DIGITAL_OUT_ACTION_SCHEMA2 = cv.All(
    {
        cv.GenerateID(): cv.use_id(MCS),
        cv.Required(CONF_DIGITAL_OUT2): cv.All(
            cv.templatable(
                cv.int_range(min = 1, max = 20)
            ),
        ),
    }
)

@automation.register_action(
    "mcs.digital_out2", 
    Digital_Out_Action2, 
    DIGITAL_OUT_ACTION_SCHEMA2
)

async def digital_out_to_code2(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    val = config[CONF_DIGITAL_OUT2]
    if cg.is_template(val):
        template_ = await cg.templatable(val, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_dig_out2(template_))

    return var

