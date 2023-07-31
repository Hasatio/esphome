from typing import Optional
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor
from esphome.const import CONF_ID

empty_binary_sensor_ns = cg.esphome_ns.namespace('empty_binary_sensor') # esphome üzerinde kullanılan komponent adı "empty_binary_sensor" 

EmptyBinarySensor = empty_binary_sensor_ns.class_('EmptyBinarySensor', binary_sensor.binary_Sensor, cg.Component)

CONFIG_SCHEMA = (
        cv.Schema(
{
    cv.GenerateID(): cv.declare_id(EmptySensor),
}).extend(cv.COMPONENT_SCHEMA)


def to_code(config): # komponentin genel fonksiyon tanımı
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config) # komponent tanımı yapıldı
