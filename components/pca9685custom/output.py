import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_CHANNEL, CONF_ID
from . import PCA9685customOutput, pca9685custom_ns

DEPENDENCIES = ["pca9685custom"]

PCA9685customChannel = pca9685custom_ns.class_("PCA9685customChannel", output.FloatOutput)
CONF_PCA9685custom_ID = "pca9685custom_id"

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.declare_id(PCA9685customChannel),
        cv.GenerateID(CONF_PCA9685custom_ID): cv.use_id(PCA9685customOutput),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=15),
    }
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_PCA9685custom_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(paren.register_channel(var))
    await output.register_output(var, config)