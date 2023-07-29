from typing import Optional #optional özelliği ekleme

import esphome.codegen as cg # "esphome/codegen.py" yeni adlandırması
import esphome.config_validation as cv # "esphome/config_validation.py" yeni adlandırması
from esphome.components import binary_sensor, sensor # esphome içindeki kulanılan komponentler
from esphome.const import ( # esphome içindeki sabit değişkenler
    CONF_ID,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
)

#DEPENDENCIES = ["i2c"]
CONF_MY_GAIN = "gain" # kişisel değişkenler
CONF_MY_BLUETOOTH = "bluetooth"
CONF_MY_SAMPLE = "sample"
CONF_MY_SAMPLE2 = "sample2"
UNIT_SAMPLE = "data/sec"
DEVICE_CLASS_PRESSURE = "sample"

myi2c_ns = cg.esphome_ns.namespace("myi2c") # esphome component adı "myi2c"
Myi2c = myi2c_ns.class_("Myi2c", binary_sensor.BinarySensor, sensor.Sensor, cg.Component) # sınıf tanımlaması

# CONFIG_SCHEMA = cv.All( # komponent içindekiler
#     cv.Schema(
#         {
#             cv.GenerateID(): cv.declare_id(Myi2c), # id tanımlaması
#             cv.Optional(CONF_MY_GAIN): cv.float_, # gain tanımlaması
#             cv.Optional(CONF_MY_BLUETOOTH): cv.string, # bluetooth tanımlaması
#             cv.Optional(CONF_MY_SAMPLE): sensor.sensor_schema( # sayaç sensör tanımlaması
#                     unit_of_measurement=UNIT_SAMPLE, # sensörün birimi
#                     accuracy_decimals=0, # sensörün sayısal gösterim şekli
#                     device_class=DEVICE_CLASS_EMPTY, # sensör sınıfı
#                     state_class=STATE_CLASS_MEASUREMENT,
#             ),
#             cv.Optional(CONF_MY_SAMPLE2): binary_sensor.binary_sensor_schema(),
#         }
#     )
#     .extend(cv.COMPONENT_SCHEMA)
# )
CONFIG_SCHEMA = binary_sensor.BINARY_SENSOR_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(Myi2c),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config): # fonksiyon tanımlaması
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config) # komponent tanımlaması
    yield sensor.register_sensor(var, config)
    yield binary_sensor.register_binary_sensor(var, config)
    
    if CONF_MY_GAIN in config:
        cg.add(var.gain(config[CONF_MY_GAIN])) # gain fonksiyonu tanımlaması
    
    if CONF_MY_BLUETOOTH in config:
        cg.add(var.bluetooth(config[CONF_MY_BLUETOOTH])) # bluetooth fonksiyonu tanımlaması

    # if CONF_MY_SAMPLE in config:
    #     conf = config[CONF_MY_SAMPLE]
    #     sens = await sensor.new_sensor(conf)
    #     cg.add(var.sample(sens)) # sayaç sensörün fonksiyonu tanımlaması
    
    # cg.add(var.sample(yield sensor.new_sensor(config[CONF_MY_SAMPLE])))
    # cg.add(var.sample(yield binary_sensor.new_binary_sensor(config[CONF_MY_SAMPLE2])))
