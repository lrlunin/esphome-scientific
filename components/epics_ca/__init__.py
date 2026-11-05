"""EPICS Channel Access (CA protocol v4.13) server for ESPHome.

Exposes ESPHome entities (sensor, binary_sensor, text_sensor, number, switch,
select, text, button) and free "soft" variables as EPICS process variables
that can be used with caget / caput / camonitor, CS-Studio / Phoebus, pyepics,
archivers, alarm handlers, etc.

Protocol reference: https://docs.epics-controls.org/en/latest/internal/ca_protocol.html
"""

import ipaddress
import re

from esphome import automation
import esphome.codegen as cg
from esphome.components import (
    binary_sensor,
    button,
    number,
    select,
    sensor,
    socket,
    switch,
    text,
    text_sensor,
)
from esphome.components.const import CONF_DESCRIPTION
import esphome.config_validation as cv
from esphome.const import (
    CONF_BINARY_SENSOR,
    CONF_BUTTON,
    CONF_HIGH,
    CONF_HYSTERESIS,
    CONF_ID,
    CONF_INITIAL_VALUE,
    CONF_LOW,
    CONF_NAME,
    CONF_NUMBER,
    CONF_PORT,
    CONF_SENSOR,
    CONF_SWITCH,
    CONF_TEXT,
    CONF_TRIGGER_ID,
    CONF_TYPE,
)

CODEOWNERS = ["@lrlunin"]
DEPENDENCIES = ["network"]
DOMAIN = "epics_ca"

CONF_ARCHIVE_DEADBAND = "archive_deadband"
CONF_BEACON_ADDRESSES = "beacon_addresses"
CONF_BEACON_PERIOD = "beacon_period"
CONF_BEACON_PORT = "beacon_port"
CONF_BEACONS_ENABLED = "beacons_enabled"
CONF_DISPLAY_HIGH = "display_high"
CONF_DISPLAY_LOW = "display_low"
CONF_DRIVE_HIGH = "drive_high"
CONF_DRIVE_LOW = "drive_low"
CONF_ENUM_SEVERITIES = "enum_severities"
CONF_ENUM_STRINGS = "enum_strings"
CONF_FIELD_ACCESS = "field_access"
CONF_HIGH_SEVERITY = "high_severity"
CONF_HIHI = "hihi"
CONF_HIHI_SEVERITY = "hihi_severity"
CONF_LOLO = "lolo"
CONF_LOLO_SEVERITY = "lolo_severity"
CONF_LOW_SEVERITY = "low_severity"
CONF_MAX_CHANNELS_PER_CLIENT = "max_channels_per_client"
CONF_MAX_CLIENTS = "max_clients"
CONF_MAX_PAYLOAD_SIZE = "max_payload_size"
CONF_MAX_SUBSCRIPTIONS_PER_CLIENT = "max_subscriptions_per_client"
CONF_MONITOR_DEADBAND = "monitor_deadband"
CONF_ON_WRITE = "on_write"
CONF_PRECISION = "precision"
CONF_PREFIX = "prefix"
CONF_PVS = "pvs"
CONF_READ_ONLY = "read_only"
CONF_RECORD_TYPE = "record_type"
CONF_SELECT = "select"
CONF_SEND_BUFFER_SIZE = "send_buffer_size"
CONF_TCP_KEEPALIVE = "tcp_keepalive"
CONF_TEXT_SENSOR = "text_sensor"
CONF_UNITS = "units"
CONF_WRITABLE = "writable"

# ---------------------------------------------------------------------------
# C++ symbols
# ---------------------------------------------------------------------------
epics_ca_ns = cg.esphome_ns.namespace(DOMAIN)
EpicsCAServer = epics_ca_ns.class_("EpicsCAServer", cg.Component)
EpicsPV = epics_ca_ns.class_("EpicsPV")
EpicsWriteTrigger = epics_ca_ns.class_(
    "EpicsWriteTrigger", automation.Trigger.template(cg.double, cg.std_string)
)
NativeType = epics_ca_ns.enum("NativeType", is_class=True)

# ---------------------------------------------------------------------------
# Channel Access limits (EPICS base db_access.h / dbDefs.h)
# ---------------------------------------------------------------------------
DEFAULT_PORT = 5064  # EPICS_CA_SERVER_PORT
DEFAULT_BEACON_PORT = 5065  # EPICS_CA_REPEATER_PORT
MAX_PV_NAME_LENGTH = 60  # PVNAME_STRINGSZ - 1
MAX_STRING_LENGTH = 39  # MAX_STRING_SIZE - 1
MAX_UNITS_LENGTH = 7  # MAX_UNITS_SIZE - 1
MAX_ENUM_STRING_LENGTH = 25  # MAX_ENUM_STRING_SIZE - 1
MAX_ENUM_STATES = 16
MAX_PRECISION = 17

TYPE_STRING = "string"
TYPE_SHORT = "short"
TYPE_FLOAT = "float"
TYPE_ENUM = "enum"
TYPE_CHAR = "char"
TYPE_LONG = "long"
TYPE_DOUBLE = "double"

TYPES = {
    TYPE_STRING: NativeType.STRING,
    TYPE_SHORT: NativeType.SHORT,
    TYPE_FLOAT: NativeType.FLOAT,
    TYPE_ENUM: NativeType.ENUM,
    TYPE_CHAR: NativeType.CHAR,
    TYPE_LONG: NativeType.LONG,
    TYPE_DOUBLE: NativeType.DOUBLE,
}
ALL_TYPES = set(TYPES)
NUMERIC_TYPES = {TYPE_SHORT, TYPE_FLOAT, TYPE_CHAR, TYPE_LONG, TYPE_DOUBLE}
NUMERIC_OR_ENUM_TYPES = NUMERIC_TYPES | {TYPE_ENUM}

SEVERITY_NO_ALARM = 0
SEVERITY_MINOR = 1
SEVERITY_MAJOR = 2
SEVERITY_INVALID = 3
SEVERITIES = {
    "NO_ALARM": SEVERITY_NO_ALARM,
    "MINOR": SEVERITY_MINOR,
    "MAJOR": SEVERITY_MAJOR,
    "INVALID": SEVERITY_INVALID,
}
validate_severity = cv.enum(SEVERITIES, upper=True)

SOURCES = [
    CONF_SENSOR,
    CONF_BINARY_SENSOR,
    CONF_TEXT_SENSOR,
    CONF_NUMBER,
    CONF_SWITCH,
    CONF_SELECT,
    CONF_TEXT,
    CONF_BUTTON,
]

# source -> (default type, allowed types, writable by default, may be writable)
SOURCE_RULES = {
    None: (TYPE_DOUBLE, ALL_TYPES, True, True),
    CONF_SENSOR: (TYPE_DOUBLE, ALL_TYPES, False, False),
    CONF_BINARY_SENSOR: (TYPE_ENUM, NUMERIC_OR_ENUM_TYPES, False, False),
    CONF_TEXT_SENSOR: (TYPE_STRING, ALL_TYPES, False, False),
    CONF_NUMBER: (TYPE_DOUBLE, NUMERIC_TYPES, True, True),
    CONF_SWITCH: (TYPE_ENUM, NUMERIC_OR_ENUM_TYPES, True, True),
    CONF_SELECT: (TYPE_ENUM, {TYPE_ENUM}, True, True),
    CONF_TEXT: (TYPE_STRING, ALL_TYPES, True, True),
    CONF_BUTTON: (TYPE_ENUM, NUMERIC_OR_ENUM_TYPES, True, True),
}

OFF_ON_STRINGS = ["OFF", "ON"]
DEFAULT_ENUM_STRINGS = {
    CONF_BINARY_SENSOR: OFF_ON_STRINGS,
    CONF_SWITCH: OFF_ON_STRINGS,
    CONF_BUTTON: ["IDLE", "PRESS"],
}

# alarm limit -> its severity, and the severity used when none is configured
ALARM_LIMITS = (
    (CONF_HIHI, CONF_HIHI_SEVERITY, SEVERITY_MAJOR),
    (CONF_HIGH, CONF_HIGH_SEVERITY, SEVERITY_MINOR),
    (CONF_LOW, CONF_LOW_SEVERITY, SEVERITY_MINOR),
    (CONF_LOLO, CONF_LOLO_SEVERITY, SEVERITY_MAJOR),
)
ALARM_KEYS = [key for limit, sev, _ in ALARM_LIMITS for key in (limit, sev)]

_NAME_RE = re.compile(r"^[A-Za-z0-9_\-+:\[\]<>;]*$")


def _name_chars(value):
    value = cv.string_strict(value)
    if not _NAME_RE.match(value):
        raise cv.Invalid(
            "EPICS names may only contain letters, digits and _ - + : [ ] < > ; "
            "(a '.' is reserved for field access such as NAME.EGU)"
        )
    return value


validate_pv_name = cv.All(_name_chars, cv.Length(min=1, max=MAX_PV_NAME_LENGTH))
validate_prefix = cv.All(_name_chars, cv.Length(max=MAX_PV_NAME_LENGTH - 1))


def _ascii(max_len):
    def validator(value):
        value = cv.string(value)
        try:
            value.encode("ascii")
        except UnicodeEncodeError as err:
            raise cv.Invalid("Only ASCII characters are allowed") from err
        if len(value) > max_len:
            raise cv.Invalid(f"Must be at most {max_len} characters long")
        return value

    return validator


def _ipv4(value):
    value = cv.string_strict(value)
    try:
        return str(ipaddress.IPv4Address(value))
    except ValueError as err:
        raise cv.Invalid(f"'{value}' is not a valid IPv4 address") from err


def _format_initial_string(value):
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


def _finalize_pv(conf):
    src = next((key for key in SOURCES if key in conf), None)
    default_type, allowed_types, writable_default, writable_allowed = SOURCE_RULES[src]
    src_name = src or "soft (no source)"

    if CONF_TYPE not in conf:
        conf[CONF_TYPE] = cv.enum(TYPES, lower=True)(default_type)
    pv_type = str(conf[CONF_TYPE])
    if pv_type not in allowed_types:
        raise cv.Invalid(
            f"type '{pv_type}' is not supported for a {src_name} PV; "
            f"allowed: {', '.join(sorted(allowed_types))}",
            [CONF_TYPE],
        )

    # writable
    if CONF_WRITABLE in conf:
        if conf[CONF_WRITABLE] and not writable_allowed:
            raise cv.Invalid(
                f"A {src_name} PV is read-only and cannot be writable", [CONF_WRITABLE]
            )
    else:
        conf[CONF_WRITABLE] = writable_default

    # alarm limits only make sense for numeric types
    if pv_type not in NUMERIC_TYPES:
        for key in ALARM_KEYS + [CONF_HYSTERESIS]:
            if key in conf and not (key == CONF_HYSTERESIS and conf[key] == 0.0):
                raise cv.Invalid(
                    f"'{key}' is only supported for numeric types "
                    f"({', '.join(sorted(NUMERIC_TYPES))})",
                    [key],
                )
    for limit, sev, _ in ALARM_LIMITS:
        if sev in conf and limit not in conf:
            raise cv.Invalid(f"'{sev}' requires '{limit}'", [sev])

    # enum strings / severities
    if pv_type != TYPE_ENUM:
        for key in (CONF_ENUM_STRINGS, CONF_ENUM_SEVERITIES):
            if key in conf:
                raise cv.Invalid(f"'{key}' requires type '{TYPE_ENUM}'", [key])
    elif CONF_ENUM_STRINGS not in conf and src in DEFAULT_ENUM_STRINGS:
        conf[CONF_ENUM_STRINGS] = list(DEFAULT_ENUM_STRINGS[src])

    # initial value only for soft PVs (soft PVs start at 0 / "" when not given)
    if src is None and CONF_INITIAL_VALUE not in conf:
        conf[CONF_INITIAL_VALUE] = "" if pv_type == TYPE_STRING else 0.0
    if CONF_INITIAL_VALUE in conf:
        if src is not None:
            raise cv.Invalid(
                f"'{CONF_INITIAL_VALUE}' is only allowed for soft PVs (PVs without an "
                "entity source)",
                [CONF_INITIAL_VALUE],
            )
        value = conf[CONF_INITIAL_VALUE]
        if pv_type == TYPE_STRING:
            value = _format_initial_string(value)
            if len(value) > MAX_STRING_LENGTH:
                raise cv.Invalid(
                    f"String values are limited to {MAX_STRING_LENGTH} characters",
                    [CONF_INITIAL_VALUE],
                )
        elif pv_type == TYPE_ENUM and isinstance(value, str):
            strings = conf.get(CONF_ENUM_STRINGS, [])
            if value not in strings:
                raise cv.Invalid(
                    f"'{value}' is not one of the {CONF_ENUM_STRINGS}",
                    [CONF_INITIAL_VALUE],
                )
            value = float(strings.index(value))
        else:
            try:
                value = float(value)
            except ValueError as err:
                raise cv.Invalid(
                    f"{CONF_INITIAL_VALUE} must be a number for type '{pv_type}'",
                    [CONF_INITIAL_VALUE],
                ) from err
        conf[CONF_INITIAL_VALUE] = value

    return conf


PV_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EpicsPV),
        cv.Required(CONF_NAME): validate_pv_name,
        cv.Optional(CONF_TYPE): cv.enum(TYPES, lower=True),
        # entity sources (at most one)
        cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_BINARY_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_TEXT_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_NUMBER): cv.use_id(number.Number),
        cv.Optional(CONF_SWITCH): cv.use_id(switch.Switch),
        cv.Optional(CONF_SELECT): cv.use_id(select.Select),
        cv.Optional(CONF_TEXT): cv.use_id(text.Text),
        cv.Optional(CONF_BUTTON): cv.use_id(button.Button),
        # record-like properties
        cv.Optional(CONF_DESCRIPTION, default=""): _ascii(MAX_STRING_LENGTH),
        cv.Optional(CONF_UNITS, default=""): _ascii(MAX_UNITS_LENGTH),
        cv.Optional(CONF_PRECISION, default=2): cv.int_range(min=0, max=MAX_PRECISION),
        cv.Optional(CONF_DISPLAY_HIGH): cv.float_,
        cv.Optional(CONF_DISPLAY_LOW): cv.float_,
        cv.Optional(CONF_DRIVE_HIGH): cv.float_,
        cv.Optional(CONF_DRIVE_LOW): cv.float_,
        cv.Optional(CONF_HIHI): cv.float_,
        cv.Optional(CONF_HIGH): cv.float_,
        cv.Optional(CONF_LOW): cv.float_,
        cv.Optional(CONF_LOLO): cv.float_,
        cv.Optional(CONF_HIHI_SEVERITY): validate_severity,
        cv.Optional(CONF_HIGH_SEVERITY): validate_severity,
        cv.Optional(CONF_LOW_SEVERITY): validate_severity,
        cv.Optional(CONF_LOLO_SEVERITY): validate_severity,
        cv.Optional(CONF_HYSTERESIS, default=0.0): cv.positive_float,
        cv.Optional(CONF_MONITOR_DEADBAND, default=0.0): cv.float_,
        cv.Optional(CONF_ARCHIVE_DEADBAND, default=0.0): cv.float_,
        cv.Optional(CONF_ENUM_STRINGS): cv.All(
            cv.ensure_list(_ascii(MAX_ENUM_STRING_LENGTH)),
            cv.Length(min=1, max=MAX_ENUM_STATES),
        ),
        cv.Optional(CONF_ENUM_SEVERITIES): cv.All(
            cv.ensure_list(validate_severity), cv.Length(min=1, max=MAX_ENUM_STATES)
        ),
        cv.Optional(CONF_WRITABLE): cv.boolean,
        cv.Optional(CONF_RECORD_TYPE): _ascii(MAX_STRING_LENGTH),
        cv.Optional(CONF_INITIAL_VALUE): cv.Any(cv.float_, cv.string),
        cv.Optional(CONF_ON_WRITE): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(EpicsWriteTrigger)}
        ),
    }
)

PV_VALIDATOR = cv.All(PV_SCHEMA, cv.has_at_most_one_key(*SOURCES), _finalize_pv)


def _validate_names(config):
    seen = set()
    for pv in config[CONF_PVS]:
        full = config[CONF_PREFIX] + pv[CONF_NAME]
        if len(full) > MAX_PV_NAME_LENGTH:
            raise cv.Invalid(
                f"PV name '{full}' (prefix + name) is longer than "
                f"{MAX_PV_NAME_LENGTH} characters"
            )
        if full in seen:
            raise cv.Invalid(f"Duplicate PV name '{full}'")
        seen.add(full)
    return config


def _consume_sockets(config):
    """Reserve the lwIP sockets of the server: ESPHome sizes the socket pool from these."""
    socket.consume_sockets(config[CONF_MAX_CLIENTS], DOMAIN)(config)
    socket.consume_sockets(1, DOMAIN, socket.SocketType.UDP)(config)
    socket.consume_sockets(1, DOMAIN, socket.SocketType.TCP_LISTEN)(config)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EpicsCAServer),
            cv.Optional(CONF_PORT, default=DEFAULT_PORT): cv.port,
            cv.Optional(CONF_BEACON_PORT, default=DEFAULT_BEACON_PORT): cv.port,
            cv.Optional(CONF_BEACONS_ENABLED, default=True): cv.boolean,
            cv.Optional(CONF_BEACON_PERIOD, default="15s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(
                    min=cv.TimePeriod(milliseconds=100),
                    max=cv.TimePeriod(seconds=300),
                ),
            ),
            cv.Optional(
                CONF_BEACON_ADDRESSES, default=["255.255.255.255"]
            ): cv.ensure_list(_ipv4),
            cv.Optional(CONF_MAX_CLIENTS, default=4): cv.int_range(min=1, max=8),
            cv.Optional(CONF_MAX_CHANNELS_PER_CLIENT, default=128): cv.int_range(
                min=1, max=4096
            ),
            cv.Optional(CONF_MAX_SUBSCRIPTIONS_PER_CLIENT, default=128): cv.int_range(
                min=1, max=4096
            ),
            cv.Optional(CONF_SEND_BUFFER_SIZE, default=8192): cv.int_range(
                min=1024, max=65536
            ),
            cv.Optional(CONF_MAX_PAYLOAD_SIZE, default=16384): cv.int_range(
                min=256, max=65536
            ),
            cv.Optional(CONF_READ_ONLY, default=False): cv.boolean,
            cv.Optional(CONF_PREFIX, default=""): validate_prefix,
            cv.Optional(CONF_FIELD_ACCESS, default=True): cv.boolean,
            cv.Optional(CONF_TCP_KEEPALIVE, default=True): cv.boolean,
            cv.Required(CONF_PVS): cv.All(
                cv.ensure_list(PV_VALIDATOR), cv.Length(min=1)
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _validate_names,
    _consume_sockets,
)


async def _pv_to_code(server, conf):
    pv = cg.new_Pvariable(conf[CONF_ID], conf[CONF_NAME])
    pv_type = str(conf[CONF_TYPE])

    cg.add(pv.set_type(conf[CONF_TYPE]))
    cg.add(pv.set_description(conf[CONF_DESCRIPTION]))
    cg.add(pv.set_units(conf[CONF_UNITS]))
    cg.add(pv.set_precision(conf[CONF_PRECISION]))
    cg.add(pv.set_writable(conf[CONF_WRITABLE]))
    cg.add(pv.set_hysteresis(conf[CONF_HYSTERESIS]))
    cg.add(pv.set_monitor_deadband(conf[CONF_MONITOR_DEADBAND]))
    cg.add(pv.set_archive_deadband(conf[CONF_ARCHIVE_DEADBAND]))

    if (display_high := conf.get(CONF_DISPLAY_HIGH)) is not None:
        cg.add(pv.set_display_high(display_high))
    if (display_low := conf.get(CONF_DISPLAY_LOW)) is not None:
        cg.add(pv.set_display_low(display_low))
    if (drive_high := conf.get(CONF_DRIVE_HIGH)) is not None:
        cg.add(pv.set_drive_high(drive_high))
    if (drive_low := conf.get(CONF_DRIVE_LOW)) is not None:
        cg.add(pv.set_drive_low(drive_low))

    if (hihi := conf.get(CONF_HIHI)) is not None:
        cg.add(pv.set_hihi(hihi, conf.get(CONF_HIHI_SEVERITY, SEVERITY_MAJOR)))
    if (high := conf.get(CONF_HIGH)) is not None:
        cg.add(pv.set_high(high, conf.get(CONF_HIGH_SEVERITY, SEVERITY_MINOR)))
    if (low := conf.get(CONF_LOW)) is not None:
        cg.add(pv.set_low(low, conf.get(CONF_LOW_SEVERITY, SEVERITY_MINOR)))
    if (lolo := conf.get(CONF_LOLO)) is not None:
        cg.add(pv.set_lolo(lolo, conf.get(CONF_LOLO_SEVERITY, SEVERITY_MAJOR)))

    for state in conf.get(CONF_ENUM_STRINGS, []):
        cg.add(pv.add_enum_string(state))
    for severity in conf.get(CONF_ENUM_SEVERITIES, []):
        cg.add(pv.add_enum_severity(severity))

    if (record_type := conf.get(CONF_RECORD_TYPE)) is not None:
        cg.add(pv.set_record_type(record_type))

    if (initial_value := conf.get(CONF_INITIAL_VALUE)) is not None:
        if pv_type == TYPE_STRING:
            cg.add(pv.set_initial_string(initial_value))
        else:
            cg.add(pv.set_initial_value(initial_value))

    if (sensor_id := conf.get(CONF_SENSOR)) is not None:
        cg.add(pv.set_sensor(await cg.get_variable(sensor_id)))
    if (binary_sensor_id := conf.get(CONF_BINARY_SENSOR)) is not None:
        cg.add(pv.set_binary_sensor(await cg.get_variable(binary_sensor_id)))
    if (text_sensor_id := conf.get(CONF_TEXT_SENSOR)) is not None:
        cg.add(pv.set_text_sensor(await cg.get_variable(text_sensor_id)))
    if (number_id := conf.get(CONF_NUMBER)) is not None:
        cg.add(pv.set_number(await cg.get_variable(number_id)))
    if (switch_id := conf.get(CONF_SWITCH)) is not None:
        cg.add(pv.set_switch(await cg.get_variable(switch_id)))
    if (select_id := conf.get(CONF_SELECT)) is not None:
        cg.add(pv.set_select(await cg.get_variable(select_id)))
    if (text_id := conf.get(CONF_TEXT)) is not None:
        cg.add(pv.set_text(await cg.get_variable(text_id)))
    if (button_id := conf.get(CONF_BUTTON)) is not None:
        cg.add(pv.set_button(await cg.get_variable(button_id)))

    cg.add(server.add_pv(pv))

    for trig_conf in conf.get(CONF_ON_WRITE, []):
        trigger = cg.new_Pvariable(trig_conf[CONF_TRIGGER_ID], pv)
        await automation.build_automation(
            trigger, [(cg.double, "x"), (cg.std_string, "s")], trig_conf
        )


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_beacon_port(config[CONF_BEACON_PORT]))
    cg.add(var.set_beacons_enabled(config[CONF_BEACONS_ENABLED]))
    cg.add(var.set_beacon_period(config[CONF_BEACON_PERIOD].total_milliseconds))
    for addr in config[CONF_BEACON_ADDRESSES]:
        octets = ipaddress.IPv4Address(addr).packed
        cg.add(var.add_beacon_address(octets[0], octets[1], octets[2], octets[3]))
    cg.add(var.set_max_clients(config[CONF_MAX_CLIENTS]))
    cg.add(var.set_max_channels_per_client(config[CONF_MAX_CHANNELS_PER_CLIENT]))
    cg.add(
        var.set_max_subscriptions_per_client(config[CONF_MAX_SUBSCRIPTIONS_PER_CLIENT])
    )
    cg.add(var.set_send_buffer_size(config[CONF_SEND_BUFFER_SIZE]))
    cg.add(var.set_max_payload_size(config[CONF_MAX_PAYLOAD_SIZE]))
    cg.add(var.set_read_only(config[CONF_READ_ONLY]))
    cg.add(var.set_prefix(config[CONF_PREFIX]))
    cg.add(var.set_field_access(config[CONF_FIELD_ACCESS]))
    cg.add(var.set_tcp_keepalive(config[CONF_TCP_KEEPALIVE]))

    for pv_conf in config[CONF_PVS]:
        await _pv_to_code(var, pv_conf)
