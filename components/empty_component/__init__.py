import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_ID

empty_component_ns = cg.esphome_ns.namespace('empty_component')
EmptyComponent = empty_component_ns.class_('EmptyComponent', output.FloatOutput, cg.Component)

CONFIG_SCHEMA = cv.Schema({cv.GenerateID(): cv.declare_id(EmptyComponent)}).extend(cv.COMPONENT_SCHEMA)
# CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend({cv.Required(CONF_ID): cv.declare_id(EmptyComponent)}).extend(cv.COMPONENT_SCHEMA)
    
def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield output.register_output(var, config)
    yield cg.register_component(var, config)
