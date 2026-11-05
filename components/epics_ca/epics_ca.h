#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT
#include "esphome/components/text/text.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

namespace esphome::epics_ca {

/// "255.255.255.255:65535" plus the terminating NUL.
static constexpr size_t PEER_STR_SIZE = 22;

/// Native DBR/DBF type of a PV. The numeric values equal the CA DBR_xxx codes.
enum class NativeType : uint8_t {
  STRING = 0,
  SHORT = 1,
  FLOAT = 2,
  ENUM = 3,
  CHAR = 4,
  LONG = 5,
  DOUBLE = 6,
};

/// Record fields that can be accessed as "<PV>.<FIELD>". The FIELD_ prefix keeps the names clear of the
/// HIGH / LOW macros of Arduino.h.
enum class PVField : uint8_t {
  FIELD_VAL,
  FIELD_NAME,
  FIELD_DESC,
  FIELD_EGU,
  FIELD_PREC,
  FIELD_HOPR,
  FIELD_LOPR,
  FIELD_DRVH,
  FIELD_DRVL,
  FIELD_HIHI,
  FIELD_HIGH,
  FIELD_LOW,
  FIELD_LOLO,
  FIELD_HYST,
  FIELD_MDEL,
  FIELD_ADEL,
  FIELD_SEVR,
  FIELD_STAT,
  FIELD_RTYP,
};

/// Non-owning list of enum state strings: either the strings of a PV or a constant table.
class EnumStrings {
 public:
  EnumStrings() = default;
  explicit EnumStrings(const std::vector<std::string> &strings) : strings_(strings.data()), count_(strings.size()) {}
  EnumStrings(const char *const *table, size_t count) : table_(table), count_(count) {}

  size_t size() const { return this->count_; }
  const char *operator[](size_t index) const {
    return this->strings_ != nullptr ? this->strings_[index].c_str() : this->table_[index];
  }

 protected:
  const std::string *strings_{nullptr};
  const char *const *table_{nullptr};
  size_t count_{0};
};

/// Snapshot of everything needed to encode any DBR type. The strings point into the PV and are only
/// valid until the PV changes, so a snapshot is encoded right after it is taken.
struct PVData {
  NativeType type{NativeType::DOUBLE};
  double num{0.0};
  const char *str{""};
  int16_t status{0};
  int16_t severity{0};
  uint32_t sec{0};
  uint32_t nsec{0};
  const char *units{""};
  int16_t precision{0};
  double upper_disp{0.0};
  double lower_disp{0.0};
  double upper_alarm{NAN};
  double upper_warning{NAN};
  double lower_warning{NAN};
  double lower_alarm{NAN};
  double upper_ctrl{0.0};
  double lower_ctrl{0.0};
  EnumStrings enum_strs;
  const char *class_name{""};
};

class EpicsCAServer;

/// One process variable (behaves like a simple EPICS record).
class EpicsPV final {
 public:
  explicit EpicsPV(std::string name) : name_(std::move(name)) {}

  // ---- configuration (called from generated code) ----
  void set_type(NativeType type) { this->type_ = type; }
  void set_description(const std::string &description) { this->description_ = description; }
  void set_units(const std::string &units) { this->units_ = units; }
  void set_precision(int16_t precision) { this->precision_ = precision; }
  void set_display_high(double value) { this->hopr_ = value; }
  void set_display_low(double value) { this->lopr_ = value; }
  void set_drive_high(double value) { this->drvh_ = value; }
  void set_drive_low(double value) { this->drvl_ = value; }
  void set_hihi(double limit, int16_t severity) {
    this->hihi_ = limit;
    this->hihi_sev_ = severity;
  }
  void set_high(double limit, int16_t severity) {
    this->high_ = limit;
    this->high_sev_ = severity;
  }
  void set_low(double limit, int16_t severity) {
    this->low_ = limit;
    this->low_sev_ = severity;
  }
  void set_lolo(double limit, int16_t severity) {
    this->lolo_ = limit;
    this->lolo_sev_ = severity;
  }
  void set_hysteresis(double hysteresis) { this->hyst_ = hysteresis; }
  void set_monitor_deadband(double deadband) { this->mdel_ = deadband; }
  void set_archive_deadband(double deadband) { this->adel_ = deadband; }
  void add_enum_string(const std::string &state) { this->enum_strs_.push_back(state); }
  void add_enum_severity(int16_t severity) { this->enum_sevs_.push_back(severity); }
  void set_writable(bool writable) { this->writable_ = writable; }
  void set_record_type(const std::string &record_type) { this->record_type_ = record_type; }
  void set_initial_value(double value) {
    this->initial_num_ = value;
    this->initial_mode_ = InitialMode::NUMBER;
  }
  void set_initial_string(const std::string &value) {
    this->initial_str_ = value;
    this->initial_mode_ = InitialMode::STRING;
  }

#ifdef USE_SENSOR
  void set_sensor(sensor::Sensor *entity) { this->sensor_ = entity; }
#endif
#ifdef USE_BINARY_SENSOR
  void set_binary_sensor(binary_sensor::BinarySensor *entity) { this->binary_sensor_ = entity; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_text_sensor(text_sensor::TextSensor *entity) { this->text_sensor_ = entity; }
#endif
#ifdef USE_NUMBER
  void set_number(number::Number *entity) { this->number_ = entity; }
#endif
#ifdef USE_SWITCH
  void set_switch(switch_::Switch *entity) { this->switch_entity_ = entity; }
#endif
#ifdef USE_SELECT
  void set_select(select::Select *entity) { this->select_ = entity; }
#endif
#ifdef USE_TEXT
  void set_text(text::Text *entity) { this->text_entity_ = entity; }
#endif
#ifdef USE_BUTTON
  void set_button(button::Button *entity) { this->button_ = entity; }
#endif

  // ---- runtime API (usable from lambdas: id(my_pv).set_value(1.5)) ----
  /// Update the value from a number (NaN marks the PV as undefined / INVALID).
  void set_value(double value);
  /// Update the value from a string (parsed for numeric/enum PVs).
  void set_string_value(const std::string &value);
  /// Mark the current value as undefined (UDF alarm, INVALID severity).
  void set_undefined();

  double get_value() const { return this->value_; }
  const std::string &get_string_value() const { return this->str_value_; }
  int16_t get_alarm_status() const { return this->status_; }
  int16_t get_alarm_severity() const { return this->severity_; }
  bool is_defined() const { return this->defined_; }
  const std::string &get_name() const { return this->name_; }
  const std::string &get_full_name() const { return this->full_name_; }
  NativeType get_type() const { return this->type_; }
  bool is_writable() const { return this->writable_; }
  const char *get_record_type() const;

  /// Called after a CA client successfully wrote to the PV: (numeric value, string value).
  template<typename F> void add_on_write_callback(F &&callback) {
    this->write_callback_.add(std::forward<F>(callback));
  }

 protected:
  friend class EpicsCAServer;

  enum class InitialMode : uint8_t { NONE, NUMBER, STRING };

  void init_(EpicsCAServer *server, const std::string &prefix);
  void setup_bindings_();
  NativeType field_native_type_(PVField field) const;
  void fill_data_(PVField field, PVData &data) const;
  uint32_t client_write_(double num, const std::string &str, bool is_string, bool integral);
  void apply_number_(double value);
  void apply_string_(const std::string &value, bool defined);
  void compute_limit_alarm_(double value, int16_t *status, int16_t *severity);
  void update_alarm_and_post_(int16_t status, int16_t severity, uint16_t value_mask);
  bool parse_number_(const std::string &text, double *out) const;
  std::string number_to_string_(double value) const;
  bool is_integer_type_() const;

  EpicsCAServer *server_{nullptr};
  std::string name_;
  std::string full_name_;
  NativeType type_{NativeType::DOUBLE};
  std::string description_;
  std::string units_;
  std::string record_type_;
  int16_t precision_{2};
  double hopr_{NAN};
  double lopr_{NAN};
  double drvh_{NAN};
  double drvl_{NAN};
  double hihi_{NAN};
  double high_{NAN};
  double low_{NAN};
  double lolo_{NAN};
  int16_t hihi_sev_{0};
  int16_t high_sev_{0};
  int16_t low_sev_{0};
  int16_t lolo_sev_{0};
  double hyst_{0.0};
  double mdel_{0.0};
  double adel_{0.0};
  std::vector<std::string> enum_strs_;
  std::vector<int16_t> enum_sevs_;
  bool writable_{false};
  InitialMode initial_mode_{InitialMode::NONE};
  double initial_num_{0.0};
  std::string initial_str_;

  // current state
  double value_{NAN};
  std::string str_value_;
  bool defined_{false};
  int16_t status_{17};   // UDF_ALARM
  int16_t severity_{3};  // INVALID_ALARM
  uint32_t sec_{0};
  uint32_t nsec_{0};
  double mlst_{NAN};
  double alst_{NAN};
  uint8_t alarm_level_{0};

  std::function<bool(double, const std::string &)> write_handler_;
  CallbackManager<void(double, const std::string &)> write_callback_;

#ifdef USE_SENSOR
  sensor::Sensor *sensor_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *binary_sensor_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *text_sensor_{nullptr};
#endif
#ifdef USE_NUMBER
  number::Number *number_{nullptr};
#endif
#ifdef USE_SWITCH
  switch_::Switch *switch_entity_{nullptr};
#endif
#ifdef USE_SELECT
  select::Select *select_{nullptr};
#endif
#ifdef USE_TEXT
  text::Text *text_entity_{nullptr};
#endif
#ifdef USE_BUTTON
  button::Button *button_{nullptr};
#endif
};

class EpicsWriteTrigger final : public Trigger<double, std::string> {
 public:
  explicit EpicsWriteTrigger(EpicsPV *pv) {
    pv->add_on_write_callback([this](double value, const std::string &text) { this->trigger(value, text); });
  }
};

struct CAHeader {
  uint16_t cmd;
  uint32_t psize;
  uint16_t dt;
  uint32_t count;
  uint32_t p1;
  uint32_t p2;
};

struct CAChannel {
  uint32_t sid;
  uint32_t cid;
  EpicsPV *pv;
  PVField field;
};

struct CASubscription {
  uint32_t subid;
  uint32_t sid;
  EpicsPV *pv;
  PVField field;
  uint16_t dbr_type;
  uint32_t count;
  uint16_t mask;
  bool pending;
};

struct CAClient {
  int fd{-1};
  char peer[PEER_STR_SIZE]{};  // "a.b.c.d:port", for logging
  std::vector<uint8_t> rx;
  std::vector<uint8_t> tx;
  size_t tx_off{0};
  std::vector<CAChannel> channels;
  std::vector<CASubscription> subs;
  uint16_t minor{0};
  uint16_t priority{0};
  bool events_off{false};
  bool closing{false};
  std::string user;
  std::string host;
};

/// EPICS Channel Access server (protocol version 4.13).
class EpicsCAServer final : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_port(uint16_t port) { this->port_ = port; }
  void set_beacon_port(uint16_t port) { this->beacon_port_ = port; }
  void set_beacons_enabled(bool enabled) { this->beacons_enabled_ = enabled; }
  void set_beacon_period(uint32_t period_ms) { this->beacon_period_ms_ = period_ms; }
  void add_beacon_address(uint8_t octet0, uint8_t octet1, uint8_t octet2, uint8_t octet3) {
    this->beacon_addrs_.push_back((uint32_t(octet0) << 24) | (uint32_t(octet1) << 16) | (uint32_t(octet2) << 8) |
                                  uint32_t(octet3));
  }
  void set_max_clients(size_t max_clients) { this->max_clients_ = max_clients; }
  void set_max_channels_per_client(size_t max_channels) { this->max_channels_ = max_channels; }
  void set_max_subscriptions_per_client(size_t max_subscriptions) { this->max_subs_ = max_subscriptions; }
  void set_send_buffer_size(size_t size) { this->send_buffer_size_ = size; }
  void set_max_payload_size(size_t size) { this->max_payload_ = size; }
  void set_read_only(bool read_only) { this->read_only_ = read_only; }
  void set_prefix(const std::string &prefix) { this->prefix_ = prefix; }
  void set_field_access(bool field_access) { this->field_access_ = field_access; }
  void set_tcp_keepalive(bool keepalive) { this->tcp_keepalive_ = keepalive; }
  void add_pv(EpicsPV *pv) { this->pvs_.push_back(pv); }

  size_t get_client_count() const { return this->clients_.size(); }

  /// Post a monitor event (DBE_xxx mask) for one field of a PV to all subscribers.
  void post_event(EpicsPV *pv, PVField field, uint16_t mask);

 protected:
  bool start_(uint32_t now);
  void stop_();
  void handle_udp_();
  void process_datagram_(const uint8_t *buf, size_t len, const void *from_addr);
  void accept_clients_();
  void read_client_(CAClient &c);
  void process_rx_(CAClient &c);
  void handle_message_(CAClient &c, const CAHeader &h, const uint8_t *raw, const uint8_t *payload);
  void handle_create_chan_(CAClient &c, const CAHeader &h, const uint8_t *payload);
  void handle_clear_channel_(CAClient &c, const CAHeader &h, const uint8_t *raw);
  void handle_read_notify_(CAClient &c, const CAHeader &h, const uint8_t *raw);
  void handle_write_(CAClient &c, const CAHeader &h, const uint8_t *raw, const uint8_t *payload);
  void handle_event_add_(CAClient &c, const CAHeader &h, const uint8_t *raw, const uint8_t *payload);
  void handle_event_cancel_(CAClient &c, const CAHeader &h, const uint8_t *raw);
  void send_error_(CAClient &c, const uint8_t *raw, uint32_t cid, uint32_t status, const char *msg);
  /// Make room for `size` more octets in the send queue; drops the client on overflow.
  bool reserve_tx_(CAClient &c, size_t size);
  void queue_message_(CAClient &c, uint16_t cmd, uint16_t dt, uint16_t count, uint32_t p1, uint32_t p2,
                      const std::vector<uint8_t> &payload);
  void queue_header_(CAClient &c, uint16_t cmd, uint16_t dt, uint32_t count, uint32_t p1, uint32_t p2);
  bool send_monitor_(CAClient &c, CASubscription &s);
  void service_pending_(CAClient &c);
  void flush_(CAClient &c);
  void send_beacons_(uint32_t now);
  CAChannel *find_channel_(CAClient &c, uint32_t sid);
  EpicsPV *find_pv_(const std::string &name, PVField *field) const;
  bool can_write_(const EpicsPV *pv, PVField field) const;

  uint16_t port_{5064};
  uint16_t beacon_port_{5065};
  bool beacons_enabled_{true};
  uint32_t beacon_period_ms_{15000};
  std::vector<uint32_t> beacon_addrs_;  // host byte order
  size_t max_clients_{4};
  size_t max_channels_{128};
  size_t max_subs_{128};
  size_t send_buffer_size_{8192};
  size_t max_payload_{16384};
  bool read_only_{false};
  std::string prefix_;
  bool field_access_{true};
  bool tcp_keepalive_{true};

  std::vector<EpicsPV *> pvs_;
  std::vector<std::unique_ptr<CAClient>> clients_;
  // Scratch buffers, reused so that encoding a reply does not allocate once they have grown.
  std::vector<uint8_t> payload_buf_;
  std::vector<uint8_t> udp_buf_;

  int udp_fd_{-1};
  int tcp_fd_{-1};
  bool started_{false};
  bool start_attempted_{false};
  uint32_t last_start_attempt_{0};
  uint32_t next_sid_{1};
  uint32_t beacon_id_{0};
  uint32_t beacon_interval_ms_{0};
  uint32_t last_beacon_ms_{0};
};

}  // namespace esphome::epics_ca

#endif  // USE_ESP32
