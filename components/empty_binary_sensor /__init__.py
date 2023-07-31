from typing import Optional
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor
from esphome.const import CONF_ID

empty_sensor_ns = cg.esphome_ns.namespace('empty_sensor') # esphome üzerinde kullanılan komponent adı "empty_binary_sensor" 

EmptySensor = empty_sensor_ns.class_('EmptySensor', sensor.Sensor, cg.Component)

CONFIG_SCHEMA = (
        cv.Schema(
{
    cv.GenerateID(): cv.declare_id(EmptySensor),
    cv.Optional("binar"): cv.float_range(),
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config): # komponentin genel fonksiyon tanımı
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config) # komponent tanımı yapıldı
    yield sensor.register_sensor(var, config) # binary_sensor tanımı yapıldı 
