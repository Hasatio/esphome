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
CONF_COMPONENT_ID = "component_id"
CONF_MY_GAIN = "gain" # esphome da kullanılan komponent değişkeni "gain"
CONF_MY_BLUETOOTH = "bluetooth" # esphome da kullanılan komponent değişkeni "bluetooth"
CONF_MY_SAMPLE = "sample" # esphome da kullanılan sensor "sample"
CONF_MY_SAMPLE_SEC = "sample_sec" # esphome da kullanılan sensor "sample_sec"
UNIT_SAMPLE = "data"
UNIT_SAMPLE_SEC = "data/sec"

component_ns = cg.esphome_ns.namespace("posture_analyzer") # esphome komponent adı "myi2c"
Posture_Analyzer = component_ns.class_("Posture_Analyzer", cg.PollingComponent) # sınıf tanımlaması

CONFIG_SCHEMA = ( # komponent içindekiler
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Posture_Analyzer), # id tanımlaması
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
        cg.add(var.set_gain(config[CONF_MY_GAIN])) # gain fonksiyonu tanımlaması
    
    if CONF_MY_BLUETOOTH in config:
        cg.add(var.set_bluetooth_name(config[CONF_MY_BLUETOOTH])) # bluetooth fonksiyonu tanımlaması

    cg.add_library("Wire", None)
    cg.add_library("SPI", None)
    cg.add_library("BluetoothSerial", None)
    cg.add_library("Adafruit BusIO", None)
    cg.add_library("Adafruit Unified Sensor", None)
    cg.add_library("Adafruit ADS1X15", None)
    cg.add_library("Adafruit ADXL345", None)
    cg.add_library("Adafruit MAX1704X", None)
    cg.add_library("ESP32 BLE Arduino", None)
    cg.add_library("UUID", None)