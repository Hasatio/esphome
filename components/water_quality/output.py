import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.const import (
    CONF_ID, 
)

from . import (
    component_ns, 
    MyComponent,
)

CODEOWNERS = ["@hasatio"]
DEPENDENCIES = ["water_quality"]

CONF_X = "x"
CONF_Y = "y"
CONF_PUMP_TYPE = "type"
CONF_PUMP_CALIBRATION = "pump_calibration"
CONF_PUMP_MODE = "pump_mode"
CONF_PUMP_DOSE = "pump_dose"

PUMP_TYPE_NULL = 0
PUMP_TYPE_DOSE = 1
PUMP_TYPE_CIRCULATION = 2
PUMP_TYPES_SUPPORTED = [PUMP_TYPE_NULL, PUMP_TYPE_DOSE, PUMP_TYPE_CIRCULATION]


PUMP_CALIBRATION_SCHEMA = cv.Schema(
    {
        # cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_CALIBRATION): cv.All(
            cv.ensure_list(
                cv.Schema(
                    {
                        cv.Required(CONF_X): cv.All(
                            cv.ensure_list(cv.uint8_t),
                            cv.Length(min=8, max=8),
                        ),
                        cv.Required(CONF_Y): cv.All(
                            cv.ensure_list(cv.uint8_t),
                            cv.Length(min=8, max=8),
                        ),
                    }
                )
            )
        ),
    }
)
                    
PUMP_TYPE_SCHEMA = cv.typed_schema(
    {
        PUMP_TYPE_NULL: cv.Any({}),
        PUMP_TYPE_DOSE: PUMP_CALIBRATION_SCHEMA,
        PUMP_TYPE_CIRCULATION: PUMP_CALIBRATION_SCHEMA,
    },
    key=CONF_PUMP_TYPE,
    default_type=PUMP_TYPE_NULL,
    int=True,
)


PumpModeAction = component_ns.class_("PumpModeAction", automation.Action)
PumpDoseAction = component_ns.class_("PumpDoseAction", automation.Action)


PUMP_MODE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        cv.Required(CONF_PUMP_MODE): (
            cv.All(
                # [cv.Any(cv.uint8_t)],
                # [cv.ensure_list(cv.uint8_t)],
                cv.templatable(cv.uint8_t),
                # cv.Length(min=0, max=3),
            )
        ),
        # cv.Required(CONF_PUMP_MODE):
        #     cv.templatable(cv.int_range()),
    }
)

PUMP_DOSE_ACTION_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.use_id(MyComponent),
        # cv.Required(CONF_DOSE): cv.templatable(
        #     cv.int_range()
        # ),
        cv.Required(CONF_PUMP_DOSE): cv.All(
                [cv.Any(cv.uint8_t)],
        ),
    }
)


@automation.register_action(
    "water_quality.pump_mode", 
    PumpModeAction, 
    PUMP_MODE_ACTION_SCHEMA
)

@automation.register_action(
    "water_quality.pump_dose", 
    PumpDoseAction, 
    PUMP_DOSE_ACTION_SCHEMA
)


async def pump_mode_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    mode = config[CONF_PUMP_MODE]
    # template_ = await cg.templatable(mode, args, cg.uint8)
    template_ = await cg.templatable(mode, args, cg.std_vector.template(cg.uint8))
    cg.add(var.set_mode(template_))
    # cg.add(var.set_mode(mode))

    return var

async def pump_dose_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    dose = config[CONF_PUMP_DOSE]
    # template_ = await cg.templatable(dose, args, cg.uint8)
    # template_ = await cg.templatable(dose, args, cg.std_vector.template(cg.uint8))
    # cg.add(var.set_dose(template_))
    cg.add(var.set_dose(dose))

    return var
