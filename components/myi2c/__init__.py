from typing import Optional #optional özelliği ekleme

import esphome.codegen as cg # "esphome/codegen.py" yeni adlandırması
import esphome.config_validation as cv # "esphome/config_validation.py" yeni adlandırması
from esphome.components import sensor, output # esphome içindeki kulanılan komponentler
from esphome.const import ( # esphome içindeki sabit değişkenler
    CONF_ID,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
)

#DEPENDENCIES = ["i2c"]
CONF_MY_GAIN = "gain" # kişisel değişkenler
CONF_MY_BLUETOOTH = "bluetooth"
CONF_MY_SAMPLE = "sample"
UNIT_SAMPLE = "data/sec"
DEVICE_CLASS_PRESSURE = "sample"

i2c_ns = cg.esphome_ns.namespace("myi2c") # esphome component adı "myi2c"
Myi2c = i2c_ns.class_("Myi2c", cg.Component) # sınıf tanımlaması

CONFIG_SCHEMA = ( # komponent içindekiler
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Myi2c), # id tanımlaması
            cv.Optional(CONF_MY_GAIN): cv.float_, # gain tanımlaması
            cv.Optional(CONF_MY_BLUETOOTH): cv.string, # bluetooth tanımlaması
            cv.Optional(CONF_MY_SAMPLE): sensor.sensor_schema( # sayaç sensör tanımlaması
                    unit_of_measurement=UNIT_SAMPLE, # sensörün birimi
                    accuracy_decimals=0, # sensörün sayısal gösterim şekli
                    device_class=DEVICE_CLASS_EMPTY, # sensör sınıfı
                    state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config): # fonksiyon tanımlaması
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config) # komponent tanımlaması
    # await sensor.register_sensor(var, config)
    
    if CONF_MY_GAIN in config:
        cg.add(var.gain(config[CONF_MY_GAIN])) # gain fonksiyonu tanımlaması
    
    if CONF_MY_BLUETOOTH in config:
        cg.add(var.bluetooth(config[CONF_MY_BLUETOOTH])) # bluetooth fonksiyonu tanımlaması

    if CONF_MY_SAMPLE in config:
        conf = config[CONF_MY_SAMPLE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.sample(sens)) # sayaç sensörün fonksiyonu tanımlaması
