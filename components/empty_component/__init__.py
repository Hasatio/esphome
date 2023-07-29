import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_ID

empty_component_ns = cg.esphome_ns.namespace('empty_component') # esphome component adı "empty_component"
EmptyComponent = empty_component_ns.class_('EmptyComponent', output.FloatOutput, cg.Component) # "EmptyComponent" sınıfı içeriğinin tanımlanması

CONFIG_SCHEMA = cv.Schema({cv.GenerateID(): cv.declare_id(EmptyComponent)}).extend(cv.COMPONENT_SCHEMA) # komponent içeriğinde olanların tanımlaması
    
def to_code(config): # genel fonksiyon
    var = cg.new_Pvariable(config[CONF_ID])
    yield output.register_output(var, config) # output komponenti tanımı
    yield cg.register_component(var, config) # genel komponent tanımı
