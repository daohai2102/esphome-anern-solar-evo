import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_DEVICE_CLASS,
    CONF_UNIT_OF_MEASUREMENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    UNIT_VOLT,
    UNIT_AMPERE,
)

from .. import CONF_ANERN_SOLAR_EVO_ID, ANERN_SOLAR_EVO_COMPONENT_SCHEMA, anern_solar_evo_ns

DEPENDENCIES = ["anern_solar_evo"]

CONF_BATTERY_UNDER_VOLTAGE = "battery_under_voltage"
CONF_CURRENT_MAX_CHARGING_CURRENT = "current_max_charging_current"
CONF_CURRENT_MAX_AC_CHARGING_CURRENT = "current_max_ac_charging_current"
CONF_BATTERY_FLOAT_VOLTAGE = "battery_float_voltage"
CONF_BATTERY_BULK_VOLTAGE = "battery_bulk_voltage"
CONF_BATTERY_REDISCHARGE_VOLTAGE = "battery_redischarge_voltage"
CONF_BATTERY_RECHARGE_VOLTAGE = "battery_recharge_voltage"

AnernSolarEvoNumber = anern_solar_evo_ns.class_("AnernSolarEvoNumber", number.Number, cg.Component)

TYPES = {
    CONF_BATTERY_UNDER_VOLTAGE: {
        "command": "PSDV%02.1f",
        "min_value": 20.0,
        "max_value": 24.0,
        "step": 0.1,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "unit_of_measurement": UNIT_VOLT,
    },
    CONF_CURRENT_MAX_CHARGING_CURRENT: {
        "command": "MNCHGC0%02.0f",
        "min_value": 10.0,
        "max_value": 90.0,
        "step": 10.0,
        "device_class": DEVICE_CLASS_CURRENT,
        "unit_of_measurement": UNIT_AMPERE,
    },
    CONF_CURRENT_MAX_AC_CHARGING_CURRENT: {
        "command": "MUCHGC0%02.0f",
        "min_value": 2.0,
        "max_value": 90.0,
        "step": 1.0,
        "device_class": DEVICE_CLASS_CURRENT,
        "unit_of_measurement": UNIT_AMPERE,
    },
    CONF_BATTERY_FLOAT_VOLTAGE: {
        "command": "PBFT%02.1f",
        "min_value": 25.0,
        "max_value": 29.0,
        "step": 0.1,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "unit_of_measurement": UNIT_VOLT,
    },
    CONF_BATTERY_BULK_VOLTAGE: {
        "command": "PCVV%02.1f",
        "min_value": 25.0,
        "max_value": 29.0,
        "step": 0.1,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "unit_of_measurement": UNIT_VOLT,
    },
    CONF_BATTERY_REDISCHARGE_VOLTAGE: {
        "command": "PBDV%02.1f",
        "min_value": 24.0,
        "max_value": 29.0,
        "step": 0.5,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "unit_of_measurement": UNIT_VOLT,
    },
    CONF_BATTERY_RECHARGE_VOLTAGE: {
        "command": "PBCV%02.1f",
        "min_value": 21.0,
        "max_value": 25.5,
        "step": 0.5,
        "device_class": DEVICE_CLASS_VOLTAGE,
        "unit_of_measurement": UNIT_VOLT,
    },
}

CONFIG_SCHEMA = ANERN_SOLAR_EVO_COMPONENT_SCHEMA.extend(
    {
      cv.Optional(type): number.number_schema(AnernSolarEvoNumber).extend(
          {
              cv.GenerateID(): cv.declare_id(AnernSolarEvoNumber),
              cv.Optional("min_value", default=conf["min_value"]): cv.float_,
              cv.Optional("max_value", default=conf["max_value"]): cv.float_,
              cv.Optional("step", default=conf["step"]): cv.float_,
              cv.Optional(
                  CONF_DEVICE_CLASS, default=conf["device_class"]
              ): cv.string,
              cv.Optional(
                  CONF_UNIT_OF_MEASUREMENT, default=conf["unit_of_measurement"]
              ): cv.string,
          }
      ).extend(cv.COMPONENT_SCHEMA)
      for type, conf in TYPES.items()    }
)

async def to_code(config):
    paren = await cg.get_variable(config[CONF_ANERN_SOLAR_EVO_ID])
    for type, conf_data in TYPES.items():
        if type in config:
            conf = config[type]
            var = await number.new_number(
                conf,
                min_value=conf["min_value"],
                max_value=conf["max_value"],
                step=conf["step"],
            )
            await cg.register_component(var, conf)
            cg.add(getattr(paren, f"set_{type}")(var))
            cg.add(var.set_parent(paren))
            cg.add(var.set_set_command(conf_data["command"]))
