import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_FREQUENCY, CONF_ID, CONF_EXTERNAL_CLOCK_INPUT

from esphome import automation
from esphome.automation import maybe_simple_id

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

CONF_VOLUME = "volume"

pca9685custom _ns = cg.esphome_ns.namespace("pca9685custom")
PCA9685customOutput = pca9685custom_ns.class_("PCA9685customOutput", cg.Component, i2c.I2CDevice)


def validate_frequency(config):
    if config[CONF_EXTERNAL_CLOCK_INPUT]:
        if CONF_FREQUENCY in config:
            raise cv.Invalid(
                "Frequency cannot be set when using an external clock input"
            )
        return config
    if CONF_FREQUENCY not in config:
        raise cv.Invalid("Frequency is required")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PCA9685customOutput),
            cv.Optional(CONF_FREQUENCY): cv.All(
                cv.frequency, cv.Range(min=23.84, max=1525.88)
            ),
            cv.Optional(CONF_EXTERNAL_CLOCK_INPUT, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x40)),
    validate_frequency,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    if CONF_FREQUENCY in config:
        cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_extclk(config[CONF_EXTERNAL_CLOCK_INPUT]))
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

PCA9685customDoseVolumeAction = pca9685custom_ns.class_("PCA9685customDoseVolumeAction", automation.Action)

PCA9685custom_DOSE_VOLUME_ACTION_SCHEMA = cv.All(
    {
        cv.Required(CONF_ID): cv.use_id(PCA9685custom),
        cv.Required(CONF_VOLUME): cv.templatable(
            cv.float_range()
        ),  # Any way to represent as proper volume (vs. raw int)
    }
)


@automation.register_action(
    "pca9685custom.dose_volume", PCA9685customDoseVolumeAction, PCA9685custom_DOSE_VOLUME_ACTION_SCHEMA
)
async def pca9685custom_dose_volume_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    template_ = await cg.templatable(config[CONF_VOLUME], args, cg.double)
    cg.add(var.set_volume(template_))

    return var
