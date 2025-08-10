import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv

from .. import CONF_ANERN_SOLAR_EVO_ID, ANERN_SOLAR_EVO_COMPONENT_SCHEMA, anern_solar_evo_ns

DEPENDENCIES = ["anern_solar_evo"]

CONF_OUTPUT_SOURCE_PRIORITY = "output_source_priority"
CONF_BATTERY_TYPE = "battery_type"

SELECT_TYPES = {
    CONF_OUTPUT_SOURCE_PRIORITY: {
        "command": "POP",
        "options": {
            "USB (Utility first)": "00",
            "SUB (Solar + Utility first)": "01",
            "SBU (Solar + Battery first)": "02",
        },
    },
    CONF_BATTERY_TYPE: {
        "command": "PBT",
        "options": {
            "AGM (Absorbent Glass Mat)": "00",
            "FLD (Flooded)": "01",
            "USE (User Defined)": "02",
            "LIB (Lithium-Ion Battery)": "03",
            "LIC (Lithium-Ion Capacitor)": "04",
            "LIP (Lithium Polymer LiPo)": "05",
        },
    },
}

AnernSolarEvoSelect = anern_solar_evo_ns.class_("AnernSolarEvoSelect", select.Select, cg.Component)

CONFIG_SCHEMA = cv.All(
    ANERN_SOLAR_EVO_COMPONENT_SCHEMA.extend(
        {
            cv.Optional(type): select.select_schema(AnernSolarEvoSelect).extend(
                cv.COMPONENT_SCHEMA
            )
            for type in SELECT_TYPES
        }
    )
)

async def to_code(config):
    paren = await cg.get_variable(config[CONF_ANERN_SOLAR_EVO_ID])
    for type, conf in SELECT_TYPES.items():
        if type in config:
            var = await select.new_select(config[type], options=list(conf["options"].keys()))
            await cg.register_component(var, config[type])
            cg.add(getattr(paren, f"set_{type}")(var))
            cg.add(var.set_parent(paren))
            cg.add(var.set_command(conf["command"]))
            for key, val in conf["options"].items():
                cg.add(var.add_option(key, val))
