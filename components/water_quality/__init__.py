import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor
from esphome.const import (
    CONF_ID, 
    CONF_DATA,
    CONF_CUSTOM,
    CONF_CALIBRATION,
) 
CODEOWNERS = ["@hasatio"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = True

CONF_X1 = "x1"
CONF_Y1 = "y1"
CONF_X2 = "x2"
CONF_Y2 = "y2"
CONF_X3 = "x3"
CONF_Y3 = "y3"
CONF_X4 = "x4"
CONF_Y4 = "y4"
CONF_PUMP_TOTAL = "pump_total"
CONF_PUMP_STATUS = "pump_status"
CONF_ANALOG_OUTPUT = "analog_output"
CONF_DOSE = "dose"

UNIT_MILILITER = "ml"
UNIT_MILILITERS_PER_MINUTE = "ml/min"

component_ns = cg.esphome_ns.namespace("water_quality")
MyComponent = component_ns.class_("MyComponent", cg.Component)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MyComponent),
            cv.Optional(CONF_CALIBRATION): cv.All(
                cv.ensure_list(
                    cv.Schema(
                        {
                            cv.Required(CONF_X1): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y1): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_X2): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y2): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_X3): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y3): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_X4): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                            cv.Required(CONF_Y4): cv.All(
                                cv.ensure_list(cv.uint8_t),
                                cv.Length(min=8, max=8),
                            ),
                        }
                    ).extend(cv.COMPONENT_SCHEMA),
                ),
                cv.Length(max=12),
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    if CONF_CALIBRATION in config:
        for conf in config[CONF_CALIBRATION]:
            cg.add(var.calibration(
                conf[CONF_X1], 
                conf[CONF_Y1],
                conf[CONF_X2], 
                conf[CONF_Y2],
                conf[CONF_X3], 
                conf[CONF_Y3],
                conf[CONF_X4], 
                conf[CONF_Y4],
                ))


DoseVolumeAction = component_ns.class_("DoseVolumeAction", automation.Action)

DOSE_VOLUME_ACTION_SCHEMA = cv.All(
    {
        # cv.Required(CONF_ID): cv.use_id(MyComponent),
        cv.GenerateID(): cv.use_id(MyComponent),
        # cv.Required(CONF_DOSE): cv.templatable(
        #     cv.int_range()
        # ),
        cv.Required(CONF_DOSE): cv.Any(
            cv.hex_uint8_t
        ),
    }
)


@automation.register_action(
    "water_quality.dose", 
    DoseVolumeAction, 
    DOSE_VOLUME_ACTION_SCHEMA
)

# async def dose_volume_to_code(config, action_id, template_arg, args):
#     paren = await cg.get_variable(config[CONF_ID])
#     var = cg.new_Pvariable(action_id, template_arg, paren)

#     template_ = await cg.templatable(config[CONF_DOSE], args, cg.uint8)
#     cg.add(var.set_data(template_))
    
async def dose_volume_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    code = config[CONF_DOSE]
    template_ = await cg.templatable(config[CONF_DOSE], args, cg.uint8)
    cg.add(var.set_data(code))

    return var
