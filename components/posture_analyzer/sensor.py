import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
)

from . import (
    component_ns, 
    Posture_Analyzer, 
    CONF_COMPONENT_ID,
    CONF_MY_SAMPLE, 
    CONF_MY_SAMPLE_SEC, 
    UNIT_SAMPLE, 
    UNIT_SAMPLE_SEC,
)

DEPENDENCIES = ["posture_analyzer"] # gerekli olan komponent, bu olmadan tanımlı sensörler kullanılamaz.

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_COMPONENT_ID): cv.use_id(Posture_Analyzer),
            cv.Optional(CONF_MY_SAMPLE): sensor.sensor_schema( # sayaç sensör tanımlaması
                unit_of_measurement=UNIT_SAMPLE, # sensörün birimi
                accuracy_decimals=0, # sensörün sayısal gösterim şekli
            )
            # .extend(cv.polling_component_schema("1ms"))
            ,
            cv.Optional(CONF_MY_SAMPLE_SEC): sensor.sensor_schema( # sayaç/saniye sensör tanımlaması
                unit_of_measurement=UNIT_SAMPLE_SEC, # sensörün birimi
                accuracy_decimals=0, # sensörün sayısal gösterim şekli
            )
            # .extend(cv.polling_component_schema("1ms"))
            ,
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_COMPONENT_ID])

    if CONF_MY_SAMPLE in config:
        conf = config[CONF_MY_SAMPLE]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.sample(sens))
    if CONF_MY_SAMPLE_SEC in config:
        conf = config[CONF_MY_SAMPLE_SEC]
        sens = await sensor.new_sensor(conf)
        cg.add(parent.sample_sec(sens))