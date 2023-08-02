import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_NONE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_SECOND,
    ICON_TIMER,
    DEVICE_CLASS_DURATION,
)

from . import myi2c_ns, Myi2c, CONF_MY_ID, CONF_MY_SAMPLE, CONF_MY_SAMPLE_SEC, UNIT_SAMPLE, UNIT_SAMPLE_SEC

DEPENDENCIES = ["myi2c"] # gerekli olan komponent, bu olmadan tanımlı sensörler kullanılamaz.

sensor_ns = cg.esphome_ns.namespace('sensor')
Sensor = sensor_ns.class_('Sensor', sensor.Sensor, cg.Nameable)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Myi2c),
            cv.Optional(CONF_MY_SAMPLE): sensor.sensor_schema( # sayaç sensör tanımlaması
                unit_of_measurement=UNIT_SAMPLE, # sensörün birimi
                accuracy_decimals=0, # sensörün sayısal gösterim şekli
                device_class=DEVICE_CLASS_EMPTY, # sensör sınıfı
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_NONE,
            ).extend(cv.polling_component_schema("10s")),
            cv.Optional(CONF_MY_SAMPLE_SEC): sensor.sensor_schema( # sayaç/saniye sensör tanımlaması
                unit_of_measurement=UNIT_SAMPLE_SEC, # sensörün birimi
                accuracy_decimals=0, # sensörün sayısal gösterim şekli
                device_class=DEVICE_CLASS_EMPTY, # sensör sınıfı
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_NONE,
            ).extend(cv.polling_component_schema("10s")),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)

# def to_code(config):
#     parent = yield cg.get_variable(config[CONF_ID])

#     if CONF_MY_SAMPLE in config:
#         sens = yield sensor.new_sensor(config[CONF_MY_SAMPLE])
#         cg.add(parent.sample(sens))
#     if CONF_MY_SAMPLE_SEC in config:
#         sens = yield sensor.new_sensor(config[CONF_MY_SAMPLE_SEC])
#         cg.add(parent.sample_sec(sens))
def to_code(config):
    paren = yield cg.get_variable(config[CONF_MY_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    
    yield sensor.register_sensor(var, config)
    
    cg.add(paren.register_sensor(var))
    if CONF_MY_SAMPLE in config:
        sens = yield sensor.new_sensor(config[CONF_MY_SAMPLE])
        cg.add(paren.sample(sens))
    if CONF_MY_SAMPLE_SEC in config:
        sens = yield sensor.new_sensor(config[CONF_MY_SAMPLE_SEC])
        cg.add(paren.sample_sec(sens))


