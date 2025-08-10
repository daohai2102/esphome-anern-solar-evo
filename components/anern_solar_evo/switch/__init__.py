import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import ICON_POWER

from .. import CONF_ANERN_SOLAR_EVO_ID, ANERN_SOLAR_EVO_COMPONENT_SCHEMA, anern_solar_evo_ns

DEPENDENCIES = ["uart"]

CONF_PV_OK_CONDITION_FOR_PARALLEL = "pv_ok_condition_for_parallel"
CONF_PV_POWER_BALANCE = "pv_power_balance"

TYPES = {
    CONF_PV_OK_CONDITION_FOR_PARALLEL: ("PPVOKC1", "PPVOKC0"),
    CONF_PV_POWER_BALANCE: ("PSPB1", "PSPB0"),
}

AnernSolarEvoSwitch = anern_solar_evo_ns.class_("AnernSolarEvoSwitch", switch.Switch, cg.Component)

PIPSWITCH_SCHEMA = switch.switch_schema(
    AnernSolarEvoSwitch, icon=ICON_POWER, block_inverted=True
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = ANERN_SOLAR_EVO_COMPONENT_SCHEMA.extend(
    {cv.Optional(type): PIPSWITCH_SCHEMA for type in TYPES}
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_ANERN_SOLAR_EVO_ID])

    for type, (on, off) in TYPES.items():
        if type in config:
            conf = config[type]
            var = await switch.new_switch(conf)
            await cg.register_component(var, conf)
            cg.add(getattr(paren, f"set_{type}_switch")(var))
            cg.add(var.set_parent(paren))
            cg.add(var.set_on_command(on))
            if off is not None:
                cg.add(var.set_off_command(off))
