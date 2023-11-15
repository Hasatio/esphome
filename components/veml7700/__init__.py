import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = True

component_ns = cg.esphome_ns.namespace("veml7700")
VEML7700 = component_ns.class_("VEML7700", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(VEML7700),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x10))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)