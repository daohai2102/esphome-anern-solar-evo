import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@andreashergert1984"]
AUTO_LOAD = ["binary_sensor", "text_sensor", "sensor", "switch", "output", "select", "number"]
MULTI_CONF = True

CONF_ANERN_SOLAR_EVO_ID = "anern_solar_evo_id"
CONF_PRIORITY_POLLING_COMMANDS = "priority_polling_commands"  # New configuration option

anern_solar_evo_ns = cg.esphome_ns.namespace("anern_solar_evo")
AnernSolarEvoComponent = anern_solar_evo_ns.class_("AnernSolarEvo", cg.Component)

ANERN_SOLAR_EVO_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ANERN_SOLAR_EVO_ID): cv.use_id(AnernSolarEvoComponent),
    }
)

# A list of valid polling commands to check against
VALID_POLLING_COMMANDS = [
    "QPIGS", "QPIRI", "QMOD", "QFLAG", "QPIWS", "QT", "QMN", "QID"
]

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(AnernSolarEvoComponent),
        # Add the new configuration option for priority commands
        cv.Optional(CONF_PRIORITY_POLLING_COMMANDS): cv.ensure_list(
            cv.All(cv.string, cv.Upper, cv.one_of(*VALID_POLLING_COMMANDS))
        ),
    })
    .extend(cv.polling_component_schema("1s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)

    # Pass the priority command list to the C++ component
    if priority_commands := config.get(CONF_PRIORITY_POLLING_COMMANDS):
        # Convert the command name string to its corresponding enum value
        command_to_enum = {
            "QPIGS": anern_solar_evo_ns.POLLING_QPIGS,
            "QPIRI": anern_solar_evo_ns.POLLING_QPIRI,
            "QMOD": anern_solar_evo_ns.POLLING_QMOD,
            "QFLAG": anern_solar_evo_ns.POLLING_QFLAG,
            "QPIWS": anern_solar_evo_ns.POLLING_QPIWS,
            "QT": anern_solar_evo_ns.POLLING_QT,
            "QMN": anern_solar_evo_ns.POLLING_QMN,
            "QID": anern_solar_evo_ns.POLLING_QID,
        }
        enum_list = [command_to_enum[cmd] for cmd in priority_commands]
        cg.add(var.set_priority_polling_commands(enum_list))
