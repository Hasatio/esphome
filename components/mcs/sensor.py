import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
)

from . import (
    mcs_ns, 
    MCS,
    CONF_COMPONENT_ID
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["mcs"]


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_COMPONENT_ID): cv.use_id(MCS),
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_COMPONENT_ID])
        