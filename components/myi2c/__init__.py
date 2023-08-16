import esphome.codegen as cg # "esphome/codegen.py" yeni adlandırması
import esphome.config_validation as cv # "esphome/config_validation.py" yeni adlandırması
from esphome.components import sensor # esphome içindeki kulanılan komponentler
from esphome.const import ( # esphome içindeki sabit değişkenler
    CONF_ID,
)

AUTO_LOAD = ["sensor"]
MULTI_CONF = True

#DEPENDENCIES = ["i2c"]

# kişisel değişkenler
CONF_MY_GAIN = "gain" # esphome da kullanılan komponent değişkeni "gain"
CONF_MY_BLUETOOTH = "bluetooth" # esphome da kullanılan komponent değişkeni "bluetooth"
CONF_MY_SAMPLE = "sample" # esphome da kullanılan sensor "sample"
CONF_MY_SAMPLE_SEC = "sample_sec" # esphome da kullanılan sensor "sample_sec"
UNIT_SAMPLE = "data"
UNIT_SAMPLE_SEC = "data/sec"

myi2c_ns = cg.esphome_ns.namespace("myi2c") # esphome komponent adı "myi2c"
Myi2c = myi2c_ns.class_("Myi2c", cg.Component) # sınıf tanımlaması

CONFIG_SCHEMA = ( # komponent içindekiler
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Myi2c), # id tanımlaması
            cv.Optional(CONF_MY_GAIN): cv.float_, # gain tanımlaması
            cv.Optional(CONF_MY_BLUETOOTH): cv.string, # bluetooth tanımlaması
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config): # fonksiyon tanımlaması
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config) # komponent tanımlaması
    
    if CONF_MY_GAIN in config:
        cg.add(var.gain(config[CONF_MY_GAIN])) # gain fonksiyonu tanımlaması
    
    if CONF_MY_BLUETOOTH in config:
        cg.add(var.bluetooth(config[CONF_MY_BLUETOOTH])) # bluetooth fonksiyonu tanımlaması

    cg.add_library(
        "Adafruit ADS1X15",
    )
    cg.add_library(
        "Adafruit ADXL345",
    )
    cg.add_library(
        "Adafruit MAX1704X",
    )
    cg.add_library(
        "BluetoothSerial",
    )
