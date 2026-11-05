# ESPHome scientific components

[![Component tests](https://img.shields.io/github/actions/workflow/status/lrlunin/esphome-scientific/component-tests.yml?branch=main&label=component%20tests&logo=github)](https://github.com/lrlunin/esphome-scientific/actions/workflows/component-tests.yml)
[![ESPHome](https://img.shields.io/badge/ESPHome-external%20components-000000?logo=esphome)](https://esphome.io/components/external_components)
[![Tested against](https://img.shields.io/badge/tested%20against-ESPHome%20dev-blue)](https://github.com/esphome/esphome/tree/dev)
[![EPICS CA](https://img.shields.io/badge/epics__ca-ESP32%20%7C%20ESP32--C3-informational)](#epics_ca--epics-channel-access-server)
[![Tango](https://img.shields.io/badge/tango-ESP32%20%7C%20ESP32--C3%20%7C%20ESP8266%20%7C%20RP2040%20%7C%20host-informational)](#tango--tango-controls-device-server)

ESPHome external components that put ESPHome entities on the control-system protocols used
at accelerators, beamlines and labs:

| Component  | Protocol                                     | Clients                                              |
| ---------- | -------------------------------------------- | ---------------------------------------------------- |
| `epics_ca` | EPICS Channel Access server (CA v4.13)       | `caget`/`caput`/`camonitor`, CS-Studio/Phoebus, pyepics, caproto, archivers |
| `tango`    | Tango Controls device server (CORBA/GIOP, IDL 6) | PyTango, Jive, Taurus, ATK, `tango_admin`         |

Neither component needs a gateway or a PC in the middle: the microcontroller speaks the protocol itself.

## Installation

```yaml
external_components:
  - source: github://lrlunin/esphome-scientific
    components: [epics_ca, tango]
```

## CI

The [component tests workflow](.github/workflows/component-tests.yml) copies each component into an
ESPHome checkout (`dev` by default) and runs ESPHome's own `script/test_build_components.py`,
first `config` then `compile`, for every target in this matrix:

| Component  | esp32-idf | esp32-ard | esp32-c3-idf | esp32-c3-ard | esp8266-ard | rp2040-ard | host |
| ---------- | :-------: | :-------: | :----------: | :----------: | :---------: | :--------: | :--: |
| `epics_ca` |     ✓     |     ✓     |      ✓       |      ✓       |      –      |     –      |  –   |
| `tango`    |     ✓     |     ✓     |      ✓       |      ✓       |      ✓      |     ✓      |  ✓   |

The test configurations live in [`tests/components/`](tests/components). These are compile tests: they do
not exercise the protocols.

---

## `epics_ca` – EPICS Channel Access server

Every PV behaves like a simple EPICS record: it has a value, alarm status and severity, a time stamp,
display/control limits, and monitors with deadbands. A PV is either bound to an ESPHome entity or is a
"soft" PV that only lives in the server.

```yaml
epics_ca:
  prefix: "LAB:ESP1:"
  pvs:
    - name: TEMPERATURE          # -> LAB:ESP1:TEMPERATURE
      sensor: temperature
      units: degC
      precision: 1
      high: 40                   # MINOR alarm above 40
      hihi: 50                   # MAJOR alarm above 50
    - name: RELAY
      switch: relay              # caput LAB:ESP1:RELAY 1
    - name: SETPOINT
      number: setpoint
      on_write:
        - logger.log:
            format: "SETPOINT written: %.2f (%s)"
            args: ["x", "s.c_str()"]
    - name: COUNTER              # soft PV, updated from a lambda
      id: counter_pv
      type: long
      drive_low: 0
      drive_high: 1000

interval:
  - interval: 60s
    then:
      - lambda: id(counter_pv).set_value(id(counter_pv).get_value() + 1);
```

```console
$ caget LAB:ESP1:TEMPERATURE LAB:ESP1:TEMPERATURE.EGU
$ camonitor LAB:ESP1:TEMPERATURE
$ caput LAB:ESP1:SETPOINT 21.5
```

### Server options

| Option                         | Default             | Description |
| ------------------------------ | ------------------- | ----------- |
| `port`                         | `5064`              | TCP and UDP port (`EPICS_CA_SERVER_PORT`). |
| `beacons_enabled`              | `true`              | Send `RSRV_IS_UP` beacons. |
| `beacon_port`                  | `5065`              | Port the beacons go to (`EPICS_CA_REPEATER_PORT`). |
| `beacon_period`                | `15s`               | Final beacon interval (100 ms – 300 s). Beacons start at 20 ms and double up to it. |
| `beacon_addresses`             | `[255.255.255.255]` | IPv4 addresses the beacons are sent to. |
| `max_clients`                  | `4`                 | Simultaneous TCP clients (1 – 8). |
| `max_channels_per_client`      | `128`               | 1 – 4096. |
| `max_subscriptions_per_client` | `128`               | 1 – 4096. |
| `send_buffer_size`             | `8192`              | Per-client send queue in bytes. A client whose backlog reaches twice this size is dropped. |
| `max_payload_size`             | `16384`             | Largest request accepted from a client. |
| `read_only`                    | `false`             | Refuse every write. |
| `prefix`                       | `""`                | Prepended to every PV name. |
| `field_access`                 | `true`              | Serve `<PV>.<FIELD>` names. |
| `tcp_keepalive`                | `true`              | TCP keepalive on client connections (60 s idle, 10 s interval, 3 probes). |
| `pvs`                          | **required**        | The PVs, see below. |

### PV options

| Option                                        | Default            | Description |
| --------------------------------------------- | ------------------ | ----------- |
| `name`                                        | **required**       | PV name. With the prefix it can be up to 60 characters. Allowed: letters, digits, `_ - + : [ ] < > ;`. |
| `sensor`, `binary_sensor`, `text_sensor`, `number`, `switch`, `select`, `text`, `button` | – | The entity the PV mirrors (at most one). Without one, the PV is a soft PV. |
| `type`                                        | depends on source  | `double`, `float`, `long`, `short`, `char`, `enum` or `string`. |
| `description`                                 | `""`               | `.DESC`, up to 39 characters. |
| `units`                                       | `""`               | `.EGU`, up to 7 characters. |
| `precision`                                   | `2`                | `.PREC`, 0 – 17. |
| `display_low` / `display_high`                | –                  | `.LOPR` / `.HOPR`. For a `number` they default to its min / max. |
| `drive_low` / `drive_high`                    | –                  | `.DRVL` / `.DRVH`: writes are clamped to these. For a `number` they default to its min / max. |
| `lolo`, `low`, `high`, `hihi`                 | –                  | Alarm limits (numeric types only). |
| `lolo_severity`, `low_severity`, `high_severity`, `hihi_severity` | `MAJOR`, `MINOR`, `MINOR`, `MAJOR` | `NO_ALARM`, `MINOR`, `MAJOR` or `INVALID`. |
| `hysteresis`                                  | `0`                | `.HYST` of the alarm limits. |
| `monitor_deadband` / `archive_deadband`       | `0`                | `.MDEL` / `.ADEL`. A negative value posts every update. |
| `enum_strings` / `enum_severities`            | –                  | Up to 16 states (25 characters each), with one severity per state. |
| `writable`                                    | depends on source  | Whether clients may write the PV. |
| `record_type`                                 | derived            | Value of `.RTYP` and `DBR_CLASS_NAME`, e.g. `ai`, `bo`, `mbbi`, `stringout`. |
| `initial_value`                               | `0` / `""`         | Soft PVs only. For `enum` PVs it may be a state name. |
| `on_write`                                    | –                  | Automation run after a successful client write. `x` is the value as a number, `s` as a string. |

| Source          | Default type | Allowed types               | Writable           |
| --------------- | ------------ | --------------------------- | ------------------ |
| none (soft PV)  | `double`     | all                         | yes (configurable) |
| `sensor`        | `double`     | all                         | no                 |
| `binary_sensor` | `enum`       | numeric, `enum`             | no                 |
| `text_sensor`   | `string`     | all                         | no                 |
| `number`        | `double`     | numeric                     | yes (configurable) |
| `switch`        | `enum`       | numeric, `enum`             | yes (configurable) |
| `select`        | `enum`       | `enum`                      | yes (configurable) |
| `text`          | `string`     | all                         | yes (configurable) |
| `button`        | `enum`       | numeric, `enum`             | yes: a non-zero write presses it |

Fields available as `<PV>.<FIELD>` (all read only except `VAL`): `VAL NAME DESC EGU PREC HOPR LOPR DRVH DRVL
HIHI HIGH LOW LOLO HYST MDEL ADEL SEVR STAT RTYP`.

Lambda API of a PV: `set_value(double)` (NaN marks it undefined), `set_string_value(std::string)`,
`set_undefined()`, `get_value()`, `get_string_value()`, `get_alarm_status()`, `get_alarm_severity()`,
`is_defined()`.

### Limitations

- **ESP32 only** (any variant, Arduino or ESP-IDF): the server uses the lwIP socket API directly.
- **IPv4 only.** Name resolution is UDP search, answered by the server itself. There is no CA name server
  and no search over TCP.
- **Scalars only.** Every PV has one element. Reads and monitors of more than one element get
  `ECA_BADCOUNT`, and writes of an array use its first element. There are no waveforms.
- **No access security.** There are no ACF/ASG rules and no host or user checks. Anyone who can reach the
  device can write every writable PV. Use `read_only: true`, `writable: false` or the network to restrict
  access.
- **No alarm acknowledgement.** `DBR_PUT_ACKT` / `DBR_PUT_ACKS` writes are accepted and have no effect, and
  `ackt` / `acks` are always 0.
- **Fields are static.** Only `VAL` can be written. Changing `HIHI`, `EGU`, … at run time is not possible,
  and no `DBE_PROPERTY` events are posted.
- **Monitor filters are ignored.** The low/high/timeout values of `EVENT_ADD` are not used. Under
  back-pressure a subscription keeps only the latest value, so intermediate updates can be dropped.
- **String sizes.** Strings are limited to 39 characters, units to 7, and enum states to 16 states of 25
  characters. A `select` with more options reports only the first 16 in `DBR_GR/CTRL_ENUM`.
- **Time stamps need a clock.** Until the system time is set (e.g. by `time: - platform: sntp`), time
  stamps are 0 (the EPICS epoch).
- **Not PV Access.** Only Channel Access is spoken, so `pvget`/`pvmonitor` (pva) will not find the PVs.
- **Limited connections.** At most 8 clients, with the per-client channel and subscription limits above.

---

## `tango` – Tango Controls device server

The node serves **one** Tango device whose attributes and commands are ESPHome entities. Clients reach it
directly, without a Tango database:

```text
tango://<esp-ip>:10000/esphome/lab/1#dbase=no
```

```yaml
tango:
  id: tango_server
  device: esphome/lab/1
  sensors:
    - id: chip_temperature
      name: ChipTemperature
      description: ESP32 internal temperature
  binary_sensors:
    - id: door
      name: DoorOpen
  switches:
    - id: relay
      name: Relay
      label: Main relay
  numbers:
    - id: setpoint
      name: SetPoint
  text_sensors:
    - id: firmware
      name: Firmware
  buttons:
    - id: restart_button
      name: Restart
```

```python
import tango

dev = tango.DeviceProxy("tango://192.168.1.106:10000/esphome/lab/1#dbase=no")
print(dev.state(), dev.ChipTemperature)
dev.Relay = True
dev.Restart()
```

### Options

| Option             | Default                              | Description |
| ------------------ | ------------------------------------ | ----------- |
| `device`           | **required**                         | Device name `domain/family/member`. |
| `port`             | `10000`                              | TCP port. |
| `device_class`     | `ESPHome`                            | Tango class name (also the server executable name). |
| `server_instance`  | the `esphome: name`                  | Server instance. The server id is `<device_class>/<server_instance>`. |
| `description`      | `A Tango device served by ESPHome`   | Device description. |
| `max_clients`      | `4`                                  | Simultaneous connections (1 – 8). When all are in use, the least recently used idle connection is closed; CORBA clients reconnect on their own. |
| `max_message_size` | `16384`                              | Largest GIOP message, fragments included (2048 – 262144). |
| `idle_timeout`     | `0s` (never)                         | Close connections that stay idle this long. |

Entity lists, each entry taking `id` (required), `name` (default: the id), `description` and `label`:

| List             | Tango type              | Access     | Extra options |
| ---------------- | ----------------------- | ---------- | ------------- |
| `sensors`        | `DevDouble` attribute   | READ       | `unit` (default: unit of measurement), `format` (default: from accuracy decimals) |
| `numbers`        | `DevDouble` attribute   | READ_WRITE | `unit`, `format`. `min_value` / `max_value` come from the number. |
| `binary_sensors` | `DevBoolean` attribute  | READ       | – |
| `switches`       | `DevBoolean` attribute  | READ_WRITE | – |
| `text_sensors`   | `DevString` attribute   | READ       | – |
| `buttons`        | `DevVoid` command       | –          | no `label` |

The built-in `State` and `Status` attributes and the `Init`, `State` and `Status` commands are always there.
`info`, `ping`, `black_box`, `command_list_query`, `get_attribute_config` and the read/write operations of IDL
versions 1 – 6 are implemented.

Lambda API: `id(tango_server).set_state(tango::DevState::ALARM)`, `set_status("...")`, `get_state()`,
`get_ior()`. From C++, `add_attribute()` / `add_command()` add attributes and commands of other scalar types.

### Registering the device in a Tango database

Through a Tango database, the device is reached by name (`tango://<db-host>:10000/esphome/lab/1`, or
just `esphome/lab/1` with `TANGO_HOST` set). The ESP does not talk to the database itself. Instead,
register it once with the IOR the ESP prints at start-up (`[I][tango]: IOR: IOR:0100…`, also shown
by `dump_config`):

```python
import tango

DEVICE = "esphome/lab/1"              # tango: device
SERVER = "ESPHome/tango-lab"          # <device_class>/<server_instance>; the instance defaults to the esphome: name
IOR = "IOR:010000001700000049444c..."  # copied from the ESP log

db = tango.Database()                 # uses $TANGO_HOST, or tango.Database("db-host", 10000)

info = tango.DbDevInfo()
info.name, info._class, info.server = DEVICE, "ESPHome", SERVER
db.add_device(info)

export = tango.DbDevExportInfo()
export.name, export.ior, export.host, export.version, export.pid = DEVICE, IOR, "tango-lab", "6", 0
db.export_device(export)

print(tango.DeviceProxy(DEVICE).state())   # now resolved through the database
```

To remove it again, call `db.unexport_device(DEVICE)` and then `db.delete_device(DEVICE)`. The IOR contains
the ESP's IP address, so give the ESP a fixed address (or a DHCP reservation) and export again if the
address changes.

### Limitations

- **One device per node, no admin device.** `adm_name` returns `dserver/<server id>`, but that device is
  not served. Anything that goes through the admin device fails: polling configuration, `RestartServer`,
  Astor/Starter, and event subscription.
- **No events.** There are no change, periodic, archive or user events (the server has no ZeroMQ), so
  clients must poll. `subscribe_event` fails.
- **No database self-registration.** The device has to be exported by hand (see above), or reached with
  `#dbase=no`.
- **Scalar attributes only.** There are no SPECTRUM/IMAGE attributes, no `DevEncoded` writes, no pipes and
  no attribute or command history.
- **Read-only configuration.** `set_attribute_config` is not supported. Attribute properties come from the
  YAML. Alarm and warning limits and event properties are always "Not specified".
- **Write semantics.** Written values are checked against the attribute type and `min_value` /
  `max_value`. There are no memorized attributes. For switches and numbers, `w_value` is the value
  read back, because Home Assistant or automations can change them too.
- **No security.** There is no TangoAccessControl and no authentication: anyone who can reach the port can
  write attributes and run commands.
- **Transport.** GIOP 1.0 – 1.2 over plain TCP (IIOP), with fragment reassembly. There is no SSL/IIOP and no
  bidirectional GIOP. The IOR prefers the node's IPv4 address.
- **Init does nothing** unless a C++ callback is registered with `device().set_init_callback()`.
- **Black box.** It keeps the last 25 operations; entries longer than 159 characters are cut.
- **Time stamps** come from the system clock, so add a `time` component (e.g. SNTP) for meaningful values.
