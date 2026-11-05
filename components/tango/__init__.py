"""Tango Controls device server for ESPHome (CORBA/GIOP, Tango IDL Device_1..Device_6).

Protocol references: https://tango-controls.readthedocs.io (tango.idl) and the OMG CORBA
specification, part 2 (GIOP/IIOP, CDR).
"""

import re

import esphome.codegen as cg
from esphome.components import (
    binary_sensor,
    button,
    number,
    sensor,
    socket,
    switch,
    text_sensor,
)
from esphome.components.const import CONF_BUTTONS, CONF_DESCRIPTION, CONF_LABEL
import esphome.config_validation as cv
from esphome.const import (
    CONF_BINARY_SENSORS,
    CONF_DEVICE,
    CONF_DEVICE_CLASS,
    CONF_FORMAT,
    CONF_ID,
    CONF_NAME,
    CONF_PORT,
    CONF_SENSORS,
    CONF_SWITCHES,
    CONF_TEXT_SENSORS,
)
from esphome.core import CORE

AUTO_LOAD = ["socket"]
CODEOWNERS = ["@lrlunin"]
DEPENDENCIES = ["network"]
DOMAIN = "tango"

CONF_IDLE_TIMEOUT = "idle_timeout"
CONF_MAX_CLIENTS = "max_clients"
CONF_MAX_MESSAGE_SIZE = "max_message_size"
CONF_NUMBERS = "numbers"
CONF_SERVER_INSTANCE = "server_instance"
CONF_UNIT = "unit"

DEFAULT_PORT = 10000  # The port Tango device servers conventionally listen on.
MAX_MEMBER_NAME_LENGTH = 255

tango_ns = cg.esphome_ns.namespace(DOMAIN)
TangoServer = tango_ns.class_("TangoServer", cg.Component)
DevState = tango_ns.enum("DevState", is_class=True)

# Entity lists that become Tango attributes (buttons become commands).
ATTRIBUTE_LISTS = (
    CONF_SENSORS,
    CONF_BINARY_SENSORS,
    CONF_SWITCHES,
    CONF_NUMBERS,
    CONF_TEXT_SENSORS,
)

# Tango names: device "domain/family/member", attributes/commands identifiers.
_DEVICE_FIELD = r"[A-Za-z0-9_\-\.]+"
_DEVICE_RE = re.compile(rf"^{_DEVICE_FIELD}/{_DEVICE_FIELD}/{_DEVICE_FIELD}$")
_SERVER_INSTANCE_RE = re.compile(rf"^{_DEVICE_FIELD}$")
_MEMBER_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_RESERVED_ATTRIBUTES = {"state", "status"}
_RESERVED_COMMANDS = {"init", "state", "status"}


def validate_device_name(value):
    value = cv.string_strict(value)
    if not _DEVICE_RE.match(value):
        raise cv.Invalid(
            "Tango device names have the form domain/family/member "
            "(letters, digits, '_', '-' and '.')"
        )
    return value


def validate_member_name(value):
    value = cv.string_strict(value)
    if not _MEMBER_RE.match(value) or len(value) > MAX_MEMBER_NAME_LENGTH:
        raise cv.Invalid(
            "Tango attribute and command names start with a letter and contain "
            f"only letters, digits and '_' (at most {MAX_MEMBER_NAME_LENGTH})"
        )
    return value


def validate_server_instance(value):
    value = cv.string_strict(value)
    if not _SERVER_INSTANCE_RE.match(value):
        raise cv.Invalid("Invalid server instance name")
    return value


def _entry_schema(entity_class, with_unit=False, with_label=True):
    schema = {
        cv.Required(CONF_ID): cv.use_id(entity_class),
        cv.Optional(CONF_NAME): validate_member_name,
        cv.Optional(CONF_DESCRIPTION, default=""): cv.string,
    }
    if with_label:
        schema[cv.Optional(CONF_LABEL, default="")] = cv.string
    if with_unit:
        schema[cv.Optional(CONF_UNIT, default="")] = cv.string
        schema[cv.Optional(CONF_FORMAT, default="")] = cv.string
    return cv.Schema(schema)


def _name_of(entry):
    """Tango name of an entry: explicit name, else the ESPHome id."""
    return entry.get(CONF_NAME) or entry[CONF_ID].id


def _validate_names(config):
    seen_attrs = set()
    for key in ATTRIBUTE_LISTS:
        for i, entry in enumerate(config.get(key, [])):
            name = _name_of(entry)
            validate_member_name(name)
            lower = name.lower()
            if lower in _RESERVED_ATTRIBUTES:
                raise cv.Invalid(
                    f"'{name}' is a built-in Tango attribute", path=[key, i]
                )
            if lower in seen_attrs:
                raise cv.Invalid(
                    f"Attribute name '{name}' is used twice (names are case-insensitive)",
                    path=[key, i],
                )
            seen_attrs.add(lower)
    seen_cmds = set()
    for i, entry in enumerate(config.get(CONF_BUTTONS, [])):
        name = _name_of(entry)
        validate_member_name(name)
        lower = name.lower()
        if lower in _RESERVED_COMMANDS:
            raise cv.Invalid(
                f"'{name}' is a built-in Tango command", path=[CONF_BUTTONS, i]
            )
        if lower in seen_cmds:
            raise cv.Invalid(
                f"Command name '{name}' is used twice", path=[CONF_BUTTONS, i]
            )
        seen_cmds.add(lower)
    return config


def _consume_sockets(config):
    """Reserve the lwIP sockets of the server: ESPHome sizes the socket pool from these."""
    socket.consume_sockets(config[CONF_MAX_CLIENTS], DOMAIN)(config)
    socket.consume_sockets(1, DOMAIN, socket.SocketType.TCP_LISTEN)(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TangoServer),
            cv.Required(CONF_DEVICE): validate_device_name,
            cv.Optional(CONF_PORT, default=DEFAULT_PORT): cv.port,
            cv.Optional(CONF_DEVICE_CLASS, default="ESPHome"): validate_server_instance,
            cv.Optional(CONF_SERVER_INSTANCE): validate_server_instance,
            cv.Optional(
                CONF_DESCRIPTION, default="A Tango device served by ESPHome"
            ): cv.string,
            cv.Optional(CONF_MAX_CLIENTS, default=4): cv.int_range(min=1, max=8),
            cv.Optional(CONF_MAX_MESSAGE_SIZE, default=16384): cv.int_range(
                min=2048, max=262144
            ),
            cv.Optional(
                CONF_IDLE_TIMEOUT, default="0s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(
                _entry_schema(sensor.Sensor, with_unit=True)
            ),
            cv.Optional(CONF_BINARY_SENSORS, default=[]): cv.ensure_list(
                _entry_schema(binary_sensor.BinarySensor)
            ),
            cv.Optional(CONF_SWITCHES, default=[]): cv.ensure_list(
                _entry_schema(switch.Switch)
            ),
            cv.Optional(CONF_NUMBERS, default=[]): cv.ensure_list(
                _entry_schema(number.Number, with_unit=True)
            ),
            cv.Optional(CONF_TEXT_SENSORS, default=[]): cv.ensure_list(
                _entry_schema(text_sensor.TextSensor)
            ),
            cv.Optional(CONF_BUTTONS, default=[]): cv.ensure_list(
                _entry_schema(button.Button, with_label=False)
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_names,
    _consume_sockets,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    instance = config.get(CONF_SERVER_INSTANCE, CORE.name)
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_device_name(config[CONF_DEVICE]))
    cg.add(var.set_device_class(config[CONF_DEVICE_CLASS]))
    # Tango server ids are "<executable>/<instance>"; the admin device is dserver/<server id>.
    cg.add(var.set_server_id(f"{config[CONF_DEVICE_CLASS]}/{instance}"))
    cg.add(var.set_description(config[CONF_DESCRIPTION]))
    cg.add(var.set_max_clients(config[CONF_MAX_CLIENTS]))
    cg.add(var.set_max_message_size(config[CONF_MAX_MESSAGE_SIZE]))
    cg.add(var.set_idle_timeout(config[CONF_IDLE_TIMEOUT]))

    for entry in config[CONF_SENSORS]:
        ent = await cg.get_variable(entry[CONF_ID])
        cg.add(
            var.add_sensor(
                ent,
                _name_of(entry),
                entry[CONF_LABEL],
                entry[CONF_DESCRIPTION],
                entry[CONF_UNIT],
                entry[CONF_FORMAT],
            )
        )
    for entry in config[CONF_BINARY_SENSORS]:
        ent = await cg.get_variable(entry[CONF_ID])
        cg.add(
            var.add_binary_sensor(
                ent, _name_of(entry), entry[CONF_LABEL], entry[CONF_DESCRIPTION]
            )
        )
    for entry in config[CONF_SWITCHES]:
        ent = await cg.get_variable(entry[CONF_ID])
        cg.add(
            var.add_switch(
                ent, _name_of(entry), entry[CONF_LABEL], entry[CONF_DESCRIPTION]
            )
        )
    for entry in config[CONF_NUMBERS]:
        ent = await cg.get_variable(entry[CONF_ID])
        cg.add(
            var.add_number(
                ent,
                _name_of(entry),
                entry[CONF_LABEL],
                entry[CONF_DESCRIPTION],
                entry[CONF_UNIT],
                entry[CONF_FORMAT],
            )
        )
    for entry in config[CONF_TEXT_SENSORS]:
        ent = await cg.get_variable(entry[CONF_ID])
        cg.add(
            var.add_text_sensor(
                ent, _name_of(entry), entry[CONF_LABEL], entry[CONF_DESCRIPTION]
            )
        )
    for entry in config[CONF_BUTTONS]:
        ent = await cg.get_variable(entry[CONF_ID])
        cg.add(var.add_button(ent, _name_of(entry), entry[CONF_DESCRIPTION]))
