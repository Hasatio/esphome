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

from . import Myi2c, CONF_MY_SAMPLE, CONF_MY_SAMPLE_SEC, UNIT_SAMPLE, UNIT_SAMPLE_SEC, CONF_MY_UPTIME

DEPENDENCIES = ["myi2c"] # gerekli olan komponent, bu olmadan tanımlı sensörler kullanılamaz.

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
            ),
            cv.Optional(CONF_MY_SAMPLE_SEC): sensor.sensor_schema( # sayaç sensör tanımlaması
                unit_of_measurement=UNIT_SAMPLE_SEC, # sensörün birimi
                accuracy_decimals=0, # sensörün sayısal gösterim şekli
                device_class=DEVICE_CLASS_EMPTY, # sensör sınıfı
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_NONE,
            ).extend(cv.polling_component_schema("1s")),
            cv.Optional(CONF_MY_UPTIME): sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            icon=ICON_TIMER,
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            device_class=DEVICE_CLASS_DURATION,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ).extend(cv.polling_component_schema("1s")),
        }
    )
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])

    if CONF_MY_SAMPLE in config:
        sens = await sensor.new_sensor(config[CONF_MY_SAMPLE])
        # await cg.register_component(sens, config)
        cg.add(parent.sample(sens))
    if CONF_MY_SAMPLE_SEC in config:
        sens = await sensor.new_sensor(config[CONF_MY_SAMPLE_SEC])
        # await cg.register_component(sens, config)
        cg.add(parent.sample_sec(sens))


