import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, output
from esphome.const import CONF_ID

DEPENDENCIES = ['i2c']

custom_ns = cg.esphome_ns.namespace('custom')
Custom = custom_ns.class_('Custom', output.FloatOutput, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Custom),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)