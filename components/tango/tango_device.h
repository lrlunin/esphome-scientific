#pragma once
// Tango device served over CORBA/GIOP, implementing the Tango IDL interfaces
// Device, Device_2 ... Device_6 (the operations a client actually sends).
//
// Wire structures follow tango.idl; their marshalling was cross-checked with
// rsTango (src/corba/tango.rs and src/corba/mod.rs, CorbaServerObject::dispatch).
//
// Apart from ESPHome's container helpers this file does not depend on ESPHome: the
// ESPHome component (tango_server.h) owns the sockets and feeds the bytes to GiopConnection.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "esphome/core/helpers.h"

#include "cdr.h"
#include "giop.h"

namespace esphome::tango {

// Tango::CmdArgType (tango_const.h / tango.idl numbering).
enum TangoDataType : int32_t {
  DEV_VOID = 0,
  DEV_BOOLEAN = 1,
  DEV_SHORT = 2,
  DEV_LONG = 3,
  DEV_FLOAT = 4,
  DEV_DOUBLE = 5,
  DEV_USHORT = 6,
  DEV_ULONG = 7,
  DEV_STRING = 8,
  DEV_STATE = 19,
  CONST_DEV_STRING = 20,
  DEV_UCHAR = 22,
  DEV_LONG64 = 23,
  DEV_ULONG64 = 24,
  DEV_ENUM = 29,
};

// Tango::DevState
enum class DevState : uint32_t {
  ON = 0,
  OFF = 1,
  CLOSE = 2,
  OPEN = 3,
  INSERT = 4,
  EXTRACT = 5,
  MOVING = 6,
  STANDBY = 7,
  FAULT = 8,
  INIT = 9,
  RUNNING = 10,
  ALARM = 11,
  DISABLE = 12,
  UNKNOWN = 13,
};
/// Number of Tango::DevState values.
static constexpr uint32_t DEV_STATE_COUNT = 14;
const char *dev_state_name(DevState state);

/// Entries kept by the black box (DeviceImpl::black_box), and the size of one entry.
static constexpr size_t BLACK_BOX_DEPTH = 25;
static constexpr size_t BLACK_BOX_ENTRY_SIZE = 160;

// Tango::AttrWriteType. Only READ and READ_WRITE are offered to users.
enum AttrWriteType : uint32_t {
  ATTR_READ = 0,
  ATTR_READ_WITH_WRITE = 1,
  ATTR_WRITE = 2,
  ATTR_READ_WRITE = 3,
};

// Tango::AttrQuality
enum AttrQuality : uint32_t {
  ATTR_VALID = 0,
  ATTR_INVALID = 1,
  ATTR_ALARM = 2,
  ATTR_CHANGING = 3,
  ATTR_WARNING = 4,
};

// Tango::ErrSeverity
enum ErrSeverity : uint32_t {
  ERR_WARN = 0,
  ERR_ERR = 1,
  ERR_PANIC = 2,
};

// Tango::DevError
struct DevError {
  std::string reason;
  ErrSeverity severity{ERR_ERR};
  std::string desc;
  std::string origin;
};
using DevErrorList = std::vector<DevError>;

// Tango::NamedDevError (MultiDevFailed entry).
struct NamedDevError {
  std::string name;
  int32_t index_in_call{0};
  DevErrorList err_list;
};

/// A scalar Tango value. Storage depends on `type`:
/// DEV_BOOLEAN -> b; DEV_SHORT/LONG/LONG64/ENUM -> i; DEV_UCHAR/USHORT/ULONG/ULONG64 -> u;
/// DEV_FLOAT/DOUBLE -> d; DEV_STRING -> s; DEV_STATE -> u (DevState value); DEV_VOID -> nothing.
struct TangoValue {
  int32_t type{DEV_VOID};
  bool b{false};
  int64_t i{0};
  uint64_t u{0};
  double d{0.0};
  std::string s;

  static TangoValue make_void() { return TangoValue(); }
  static TangoValue make_bool(bool v);
  static TangoValue make_int(int32_t type, int64_t v);    // DEV_SHORT, DEV_LONG, DEV_LONG64, DEV_ENUM
  static TangoValue make_uint(int32_t type, uint64_t v);  // DEV_UCHAR, DEV_USHORT, DEV_ULONG, DEV_ULONG64
  static TangoValue make_float(int32_t type, double v);   // DEV_FLOAT, DEV_DOUBLE
  static TangoValue make_string(const std::string &v);
  static TangoValue make_state(DevState v);

  bool is_numeric() const;
  bool to_double(double &out) const;
};

/// What a read handler gives back.
struct AttrReading {
  bool valid{false};  // false: quality ATTR_INVALID and no data is sent.
  AttrQuality quality{ATTR_VALID};
  TangoValue value;
};

/// One scalar attribute.
struct TangoAttribute {
  std::string name;
  int32_t data_type{DEV_DOUBLE};
  AttrWriteType writable{ATTR_READ};
  std::string description;  // empty -> "No description"
  std::string label;        // empty -> the attribute name
  std::string unit;
  std::string format;                    // empty -> type dependent default
  std::string min_value;                 // empty -> "Not specified"
  std::string max_value;                 // empty -> "Not specified"
  std::vector<std::string> enum_labels;  // DEV_ENUM only
  std::function<void(AttrReading &)> reader;
  /// Apply a written value (already converted to `data_type` and range checked).
  /// Leave `err` empty on success.
  std::function<void(const TangoValue &, DevErrorList &)> writer;

  /// true: w_value of a READ_WRITE attribute is the last value written through Tango
  /// (cppTango semantics). false: w_value is the value read (for entities that can
  /// also be changed by other means, e.g. from Home Assistant).
  bool track_set_point{true};
  // Last written value (the "set point" returned as w_value of READ_WRITE attributes).
  TangoValue set_point;
  bool has_set_point{false};
  // Internal: the built-in State attribute (sent as the DEVICE_STATE union case).
  bool device_state{false};
};

/// One command.
struct TangoCommand {
  std::string name;
  int32_t in_type{DEV_VOID};
  int32_t out_type{DEV_VOID};
  std::string in_desc;   // empty -> "Uninitialised"
  std::string out_desc;  // empty -> "Uninitialised"
  /// `in` is already checked against `in_type`; fill `out` (type `out_type`), or `err`.
  std::function<void(const TangoValue &in, TangoValue &out, DevErrorList &err)> handler;
};

class TangoDevice final {
 public:
  TangoDevice();

  // ---- Identity / DevInfo ---- //
  void set_name(const std::string &name);
  const std::string &get_name() const { return this->name_; }
  void set_class_name(const std::string &v) { this->class_name_ = v; }
  void set_server_id(const std::string &v) { this->server_id_ = v; }
  void set_server_host(const std::string &v) { this->server_host_ = v; }
  void set_description(const std::string &v) { this->description_ = v; }
  void set_doc_url(const std::string &v) { this->doc_url_ = v; }
  void set_dev_type(const std::string &v) { this->dev_type_ = v; }
  void add_version_info(const std::string &key, const std::string &value) {
    this->version_info_.emplace_back(key, value);
  }

  // ---- State / Status ---- //
  void set_state(DevState state) { this->state_ = state; }
  DevState get_state() const { return this->state_; }
  /// An empty status means the Tango default "The device is in <STATE> state."
  void set_status(const std::string &status) { this->status_ = status; }
  std::string get_status() const;
  void set_init_callback(std::function<void()> cb) { this->on_init_ = std::move(cb); }

  // ---- Interface ---- //
  /// Returns false if the name is empty or already used (case-insensitively).
  bool add_attribute(const TangoAttribute &attr);
  bool add_command(const TangoCommand &cmd);
  TangoAttribute *find_attribute(const std::string &name);

  // ---- Protocol ---- //
  bool matches_key(const std::string &key) const;
  /// Answer one GIOP Request, appending the reply (if one is expected) to `tx`.
  void handle_request(const GiopRequest &req, const std::string &peer, std::vector<uint8_t> &tx);
  /// Answer one GIOP LocateRequest.
  void handle_locate(const GiopRequest &req, std::vector<uint8_t> &tx);

  /// Stringified IOR ("IOR:...") of this device, reachable at host:port.
  std::string build_ior(const std::string &host, uint16_t port) const;

 protected:
  enum class OpResult { OK, DEV_FAILED, MULTI_DEV_FAILED, BAD_OPERATION, MARSHAL, NO_IMPLEMENT };

  struct OpContext {
    CdrReader in;
    CdrWriter *out;
    DevErrorList err;
    std::vector<NamedDevError> multi;
    std::string bb_detail;  // Extra black box detail ("cmd = State").
  };

  OpResult dispatch_(const std::string &op, OpContext &ctx);

  OpResult op_info_(OpContext &ctx, int version);
  OpResult op_black_box_(OpContext &ctx);
  OpResult op_command_inout_(OpContext &ctx, int version);
  OpResult op_command_query_(OpContext &ctx, int version);
  OpResult op_command_list_query_(OpContext &ctx, int version);
  OpResult op_read_attributes_(OpContext &ctx, int version);
  OpResult op_write_attributes_(OpContext &ctx, int version);
  OpResult op_write_read_attributes_(OpContext &ctx, int version);
  OpResult op_get_attribute_config_(OpContext &ctx, int version);

  // Attribute list helpers: indices [0, attrs_.size()) are user attributes, then State, Status.
  size_t attr_count_() const { return this->attrs_.size() + 2; }
  TangoAttribute *attr_at_(size_t index);
  TangoAttribute *lookup_attr_(const std::string &name);
  bool read_names_(CdrReader &in, std::vector<std::string> &names, bool &all);

  // Commands: built-ins (Init, State, Status) then user commands.
  const TangoCommand *lookup_command_(const std::string &name) const;
  void for_each_command_(const std::function<void(const TangoCommand &)> &fn) const;

  void read_attr_(TangoAttribute &attr, AttrReading &reading);
  bool write_attr_(TangoAttribute &attr, const std::vector<TangoValue> &values, DevErrorList &err);
  void write_attr_value_(CdrWriter &w, int version, TangoAttribute *attr, const std::string &name,
                         const AttrReading *reading, const DevErrorList *err);
  void write_attr_config_(CdrWriter &w, int version, const TangoAttribute &attr);
  void write_cmd_info_(CdrWriter &w, int version, const TangoCommand &cmd);

  void record_black_box_(const std::string &op, const std::string &detail, const std::string &peer);

  std::string name_;
  std::string key_;  // Lower-cased name: the CORBA object key.
  std::string class_name_{"ESPHome"};
  std::string server_id_{"ESPHome/esphome"};
  std::string server_host_{"esphome"};
  std::string description_{"A Tango device served by ESPHome"};
  std::string doc_url_{"Doc URL = https://esphome.io"};
  std::string dev_type_{"Uninitialised"};
  std::vector<std::pair<std::string, std::string>> version_info_;

  DevState state_{DevState::ON};
  std::string status_;
  std::function<void()> on_init_;

  std::vector<TangoAttribute> attrs_;
  TangoAttribute state_attr_;
  TangoAttribute status_attr_;
  std::vector<TangoCommand> commands_;
  std::vector<TangoCommand> builtin_commands_;

  /// One black box line, NUL terminated (truncated if longer).
  struct BlackBoxEntry {
    char text[BLACK_BOX_ENTRY_SIZE];
  };
  StaticRingBuffer<BlackBoxEntry, BLACK_BOX_DEPTH> black_box_;  // Oldest first.
  /// Reused for every reply so that answering a request does not allocate once it has grown.
  GiopReplyBuilder reply_;
};

/// One client connection: splits the TCP byte stream into GIOP messages,
/// reassembles fragments and dispatches requests to the device.
class GiopConnection final {
 public:
  GiopConnection(TangoDevice *device, size_t max_message_size) : device_(device), max_message_(max_message_size) {}

  void set_peer(const std::string &peer) { this->peer_ = peer; }
  const std::string &peer() const { return this->peer_; }
  /// GIOP minor version of the last message received (for an orderly CloseConnection).
  uint8_t last_minor() const { return this->last_minor_; }
  /// True while a fragmented request is being received.
  bool busy() const { return this->frag_active_ || !this->rx_.empty(); }

  /// Feed received octets. Replies are appended to `tx`. Returns false when the
  /// connection must be closed (after `tx` has been sent).
  bool feed(const uint8_t *data, size_t len, std::vector<uint8_t> &tx);

 protected:
  bool process_(const GiopHeader &h, const uint8_t *msg, size_t len, std::vector<uint8_t> &tx);
  bool process_request_(const GiopHeader &h, const uint8_t *msg, size_t len, std::vector<uint8_t> &tx);

  TangoDevice *device_;
  size_t max_message_;
  std::string peer_;
  std::vector<uint8_t> rx_;
  // Fragmented request being reassembled (GIOP 1.1 / 1.2).
  std::vector<uint8_t> frag_;
  bool frag_active_{false};
  uint8_t frag_minor_{0};
  uint32_t frag_request_id_{0};
  uint8_t last_minor_{2};
};

}  // namespace esphome::tango
