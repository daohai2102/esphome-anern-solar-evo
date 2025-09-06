import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import ICON_POWER

from .. import CONF_ANERN_SOLAR_EVO_ID, ANERN_SOLAR_EVO_COMPONENT_SCHEMA, anern_solar_evo_ns

DEPENDENCIES = ["uart"]

CONF_ALARM_ON_WHEN_PRIMARY_SOURCE_INTERRUPT = "alarm_on_when_primary_source_interrupt"
CONF_OVERLOAD_RESTART_FUNCTION = "overload_restart_function"
CONF_SILENCE_BUZZER_OPEN_BUZZER = "silence_buzzer_open_buzzer"
CONF_BACKLIGHT_ON = "backlight_on"
CONF_LCD_ESCAPE_TO_DEFAULT = "lcd_escape_to_default"
CONF_OVER_TEMPERATURE_RESTART_FUNCTION = "over_temperature_restart_function"

TYPES = {
  CONF_ALARM_ON_WHEN_PRIMARY_SOURCE_INTERRUPT: ("PEy", "PDy", "QFLAG"),
  CONF_OVERLOAD_RESTART_FUNCTION: ("PEu", "PDu", "QFLAG"),
  CONF_SILENCE_BUZZER_OPEN_BUZZER: ("PEa", "PDa", "QFLAG"),
  CONF_BACKLIGHT_ON: ("PEx", "PDx", "QFLAG"),
  CONF_LCD_ESCAPE_TO_DEFAULT: ("PEk", "PDk", "QFLAG"),
  CONF_OVER_TEMPERATURE_RESTART_FUNCTION: ("PEv", "PDv", "QFLAG"),
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

    for type, (on, off, query) in TYPES.items():
        if type in config:
            conf = config[type]
            var = await switch.new_switch(conf)
            await cg.register_component(var, conf)
            cg.add(getattr(paren, f"set_{type}")(var))
            cg.add(var.set_parent(paren))
            cg.add(var.set_on_command(on))
            if off is not None:
                cg.add(var.set_off_command(off))
            cg.add(var.set_query_command(query))
