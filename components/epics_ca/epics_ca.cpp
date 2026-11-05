#include "epics_ca.h"

#ifdef USE_ESP32

#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cfloat>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <sys/time.h>

// lwIP must be included last: it defines function-like compatibility macros.
#include <lwip/inet.h>
#include <lwip/sockets.h>

// ESPHome releases before 2026.10.0 do not provide the macro.
#ifndef ESPHOME_LOG_TAG
#define ESPHOME_LOG_TAG(name, tag) static const char *const name = tag
#endif

namespace esphome::epics_ca {

ESPHOME_LOG_TAG(TAG, "epics_ca");

// ---------------------------------------------------------------------------
// Channel Access protocol constants (EPICS base caProto.h / db_access.h / caerr.h)
// ---------------------------------------------------------------------------
static constexpr uint16_t CA_MINOR_PROTOCOL_REVISION = 13;

static constexpr uint16_t CA_PROTO_VERSION = 0;
static constexpr uint16_t CA_PROTO_EVENT_ADD = 1;
static constexpr uint16_t CA_PROTO_EVENT_CANCEL = 2;
static constexpr uint16_t CA_PROTO_WRITE = 4;
static constexpr uint16_t CA_PROTO_SEARCH = 6;
static constexpr uint16_t CA_PROTO_EVENTS_OFF = 8;
static constexpr uint16_t CA_PROTO_EVENTS_ON = 9;
static constexpr uint16_t CA_PROTO_READ_SYNC = 10;
static constexpr uint16_t CA_PROTO_ERROR = 11;
static constexpr uint16_t CA_PROTO_CLEAR_CHANNEL = 12;
static constexpr uint16_t CA_PROTO_RSRV_IS_UP = 13;
static constexpr uint16_t CA_PROTO_NOT_FOUND = 14;
static constexpr uint16_t CA_PROTO_READ_NOTIFY = 15;
static constexpr uint16_t CA_PROTO_CREATE_CHAN = 18;
static constexpr uint16_t CA_PROTO_WRITE_NOTIFY = 19;
static constexpr uint16_t CA_PROTO_CLIENT_NAME = 20;
static constexpr uint16_t CA_PROTO_HOST_NAME = 21;
static constexpr uint16_t CA_PROTO_ACCESS_RIGHTS = 22;
static constexpr uint16_t CA_PROTO_ECHO = 23;
static constexpr uint16_t CA_PROTO_CREATE_CH_FAIL = 26;

static constexpr uint16_t SEARCH_DO_REPLY = 10;

/// Message header, and the extended header used for large payloads / counts.
static constexpr size_t CA_HEADER_SIZE = 16;
static constexpr size_t CA_EXTENDED_HEADER_SIZE = 24;
/// postsize value announcing an extended header (together with count == 0).
static constexpr uint32_t CA_EXTENDED_POSTSIZE = 0xFFFF;
/// Largest element count that fits the 16-bit count field of a plain header.
static constexpr uint32_t CA_MAX_SHORT_COUNT = 0xFFFF;
/// Payloads are padded to a multiple of this size.
static constexpr size_t CA_PAYLOAD_ALIGNMENT = 8;
/// Search reply payload: server minor protocol version, padded.
static constexpr size_t CA_SEARCH_REPLY_PAYLOAD_SIZE = 8;
/// Search reply address field: ~0 tells the client to use the source address of the reply.
static constexpr uint32_t CA_SEARCH_REPLY_USE_SOURCE_ADDR = 0xFFFFFFFFu;
/// EVENT_ADD payload: float low, float high, float timeout, uint16 mask, uint16 pad.
static constexpr size_t CA_EVENT_ADD_MASK_OFFSET = 12;
static constexpr size_t CA_EVENT_ADD_MIN_PAYLOAD = CA_EVENT_ADD_MASK_OFFSET + 2;

static constexpr uint32_t CA_ACCESS_READ = 1;
static constexpr uint32_t CA_ACCESS_WRITE = 2;

static constexpr uint16_t DBE_VALUE = 1;
static constexpr uint16_t DBE_LOG = 2;
static constexpr uint16_t DBE_ALARM = 4;

static constexpr uint16_t DBR_STRING = 0;
static constexpr uint16_t DBR_SHORT = 1;
static constexpr uint16_t DBR_FLOAT = 2;
static constexpr uint16_t DBR_ENUM = 3;
static constexpr uint16_t DBR_CHAR = 4;
static constexpr uint16_t DBR_LONG = 5;
static constexpr uint16_t DBR_DOUBLE = 6;
static constexpr uint16_t DBR_CTRL_DOUBLE = 34;
static constexpr uint16_t DBR_PUT_ACKT = 35;
static constexpr uint16_t DBR_PUT_ACKS = 36;
static constexpr uint16_t DBR_STSACK_STRING = 37;
static constexpr uint16_t DBR_CLASS_NAME = 38;

/// DBR types 0..34 are <class> * 7 + <base type>.
static constexpr uint16_t DBR_TYPES_PER_CLASS = 7;
static constexpr uint16_t DBR_CLASS_STS = 1;
static constexpr uint16_t DBR_CLASS_TIME = 2;
static constexpr uint16_t DBR_CLASS_GR = 3;
static constexpr uint16_t DBR_CLASS_CTRL = 4;
/// Number of limits in a DBR_GR_xxx (6) and a DBR_CTRL_xxx (8) structure.
static constexpr int DBR_GR_LIMIT_COUNT = 6;
static constexpr int DBR_CTRL_LIMIT_COUNT = 8;

static constexpr size_t MAX_STRING_SIZE = 40;
static constexpr size_t MAX_STRING_CHARS = MAX_STRING_SIZE - 1;
static constexpr size_t MAX_UNITS_SIZE = 8;
static constexpr size_t MAX_ENUM_STRING_SIZE = 26;
static constexpr size_t MAX_ENUM_STATES = 16;
static constexpr int MAX_PRECISION = 17;

// ECA status codes: (number << 3) | severity
static constexpr uint32_t ECA_NORMAL = 1;
static constexpr uint32_t ECA_ALLOCMEM = 48;
static constexpr uint32_t ECA_BADTYPE = 114;
static constexpr uint32_t ECA_PUTFAIL = 160;
static constexpr uint32_t ECA_BADCOUNT = 176;
static constexpr uint32_t ECA_BADMONID = 242;
static constexpr uint32_t ECA_NOWTACCESS = 376;
static constexpr uint32_t ECA_BADCHID = 410;

// Alarm severities / status (alarm.h)
static constexpr int16_t SEV_NO_ALARM = 0;
static constexpr int16_t SEV_INVALID = 3;
static constexpr int16_t STAT_NO_ALARM = 0;
static constexpr int16_t STAT_HIHI = 3;
static constexpr int16_t STAT_HIGH = 4;
static constexpr int16_t STAT_LOLO = 5;
static constexpr int16_t STAT_LOW = 6;
static constexpr int16_t STAT_STATE = 7;
static constexpr int16_t STAT_UDF = 17;

static constexpr uint8_t LEVEL_NONE = 0;
static constexpr uint8_t LEVEL_HIHI = 1;
static constexpr uint8_t LEVEL_LOLO = 2;
static constexpr uint8_t LEVEL_HIGH = 3;
static constexpr uint8_t LEVEL_LOW = 4;

static constexpr uint32_t POSIX_TIME_AT_EPICS_EPOCH = 631152000UL;  // 1990-01-01 00:00:00 UTC
static constexpr uint32_t NANOSECONDS_PER_MICROSECOND = 1000;

// ---------------------------------------------------------------------------
// Server tuning
// ---------------------------------------------------------------------------
static constexpr size_t MAX_UDP_SEND = 1024;
static constexpr size_t UDP_RECV_BUFFER = 1500;
static constexpr size_t TCP_RECV_CHUNK = 512;
/// Work done per loop() call, so that a busy network cannot hold the main loop.
static constexpr int MAX_DATAGRAMS_PER_LOOP = 8;
static constexpr int MAX_ACCEPTS_PER_LOOP = 4;
static constexpr int MAX_TCP_READS_PER_LOOP = 8;
/// Sent octets are dropped from the front of the send queue once they reach this size.
static constexpr size_t TX_COMPACT_THRESHOLD = 2048;
/// Longest user / host name kept from CLIENT_NAME / HOST_NAME.
static constexpr size_t MAX_CLIENT_NAME_LENGTH = 64;
static constexpr uint32_t START_RETRY_INTERVAL_MS = 5000;
/// Beacons start at this interval and double up to the configured beacon period.
static constexpr uint32_t BEACON_INITIAL_INTERVAL_MS = 20;
static constexpr int TCP_KEEPALIVE_IDLE_S = 60;
static constexpr int TCP_KEEPALIVE_INTERVAL_S = 10;
static constexpr int TCP_KEEPALIVE_COUNT = 3;
/// Large enough for any number formatted by format_double() / format_integer().
static constexpr size_t FORMAT_BUFFER_SIZE = 64;
/// "255.255.255.255" plus the terminating NUL.
static constexpr size_t IP4_STR_SIZE = 16;

static constexpr const char *ERR_BAD_CHANNEL_ID = "Invalid server channel ID";
static constexpr const char *ERR_BAD_DBR_TYPE = "Invalid DBR type";
static constexpr const char *ERR_BAD_COUNT = "Element count exceeds native count (1)";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static const char *const SEVERITY_STRINGS[] = {"NO_ALARM", "MINOR", "MAJOR", "INVALID"};

static const char *const STATUS_STRINGS[] = {
    "NO_ALARM", "READ", "WRITE", "HIHI", "HIGH", "LOLO",    "LOW", "STATE",   "COS",  "COMM",        "TIMEOUT",
    "HWLIMIT",  "CALC", "SCAN",  "LINK", "SOFT", "BAD_SUB", "UDF", "DISABLE", "SIMM", "READ_ACCESS", "WRITE_ACCESS"};

struct FieldName {
  const char *name;
  PVField field;
};

static const FieldName FIELD_NAMES[] = {
    {"VAL", PVField::FIELD_VAL},   {"NAME", PVField::FIELD_NAME}, {"DESC", PVField::FIELD_DESC},
    {"EGU", PVField::FIELD_EGU},   {"PREC", PVField::FIELD_PREC}, {"HOPR", PVField::FIELD_HOPR},
    {"LOPR", PVField::FIELD_LOPR}, {"DRVH", PVField::FIELD_DRVH}, {"DRVL", PVField::FIELD_DRVL},
    {"HIHI", PVField::FIELD_HIHI}, {"HIGH", PVField::FIELD_HIGH}, {"LOW", PVField::FIELD_LOW},
    {"LOLO", PVField::FIELD_LOLO}, {"HYST", PVField::FIELD_HYST}, {"MDEL", PVField::FIELD_MDEL},
    {"ADEL", PVField::FIELD_ADEL}, {"SEVR", PVField::FIELD_SEVR}, {"STAT", PVField::FIELD_STAT},
    {"RTYP", PVField::FIELD_RTYP},
};

template<typename T> static T saturate(double v) {
  if (std::isnan(v))
    return T(0);
  const double lo = static_cast<double>(std::numeric_limits<T>::lowest());
  const double hi = static_cast<double>(std::numeric_limits<T>::max());
  if (v <= lo)
    return std::numeric_limits<T>::lowest();
  if (v >= hi)
    return std::numeric_limits<T>::max();
  return static_cast<T>(v);  // truncation toward zero, like EPICS dbConvert
}

static float to_f32(double v) {
  if (std::isnan(v))
    return NAN;
  if (v > static_cast<double>(FLT_MAX))
    return INFINITY;
  if (v < -static_cast<double>(FLT_MAX))
    return -INFINITY;
  return static_cast<float>(v);
}

static bool is_floating(NativeType t) { return t == NativeType::FLOAT || t == NativeType::DOUBLE; }

static uint16_t rd16(const uint8_t *p) { return static_cast<uint16_t>((uint16_t(p[0]) << 8) | uint16_t(p[1])); }
static uint32_t rd32(const uint8_t *p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

static size_t padded_payload_size(size_t size) {
  return (size + CA_PAYLOAD_ALIGNMENT - 1) & ~(CA_PAYLOAD_ALIGNMENT - 1);
}

static void epics_now(uint32_t *sec, uint32_t *nsec) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const int64_t s = static_cast<int64_t>(tv.tv_sec) - static_cast<int64_t>(POSIX_TIME_AT_EPICS_EPOCH);
  if (s < 0) {  // clock not synchronised yet
    *sec = 0;
    *nsec = 0;
    return;
  }
  *sec = static_cast<uint32_t>(s);
  *nsec = static_cast<uint32_t>(tv.tv_usec) * NANOSECONDS_PER_MICROSECOND;
}

/// Format `v` with `prec` decimals into `buf` (FORMAT_BUFFER_SIZE octets) and return it.
static const char *format_double(char *buf, double v, int prec) {
  if (std::isnan(v))
    return "nan";
  if (std::isinf(v))
    return v > 0 ? "inf" : "-inf";
  prec = std::max(0, std::min(prec, MAX_PRECISION));
  if (std::fabs(v) >= 1e15) {
    snprintf(buf, FORMAT_BUFFER_SIZE, "%.*e", prec, v);
  } else {
    snprintf(buf, FORMAT_BUFFER_SIZE, "%.*f", prec, v);
  }
  return buf;
}

/// Format `v` as a (saturated) 32 bit integer into `buf` (FORMAT_BUFFER_SIZE octets) and return it.
static const char *format_integer(char *buf, double v) {
  snprintf(buf, FORMAT_BUFFER_SIZE, "%" PRId32, saturate<int32_t>(v));
  return buf;
}

/// String form of a value of the given native type; `buf` (FORMAT_BUFFER_SIZE octets) holds formatted numbers.
static const char *native_to_string(char *buf, NativeType type, double num, const char *str, int precision,
                                    const EnumStrings &strs) {
  switch (type) {
    case NativeType::STRING:
      return str;
    case NativeType::ENUM: {
      const uint16_t idx = saturate<uint16_t>(num);
      if (idx < strs.size())
        return strs[idx];
      return format_integer(buf, idx);
    }
    case NativeType::FLOAT:
    case NativeType::DOUBLE:
      return format_double(buf, num, precision);
    default:
      return format_integer(buf, num);
  }
}

/// Parse a whole string (surrounding white space allowed) as a number.
static bool parse_double(const char *in, double *out) {
  while (isspace(static_cast<unsigned char>(*in)))
    in++;
  if (*in == '\0')
    return false;
  char *end = nullptr;
  const double v = strtod(in, &end);
  if (end == in)
    return false;
  while (isspace(static_cast<unsigned char>(*end)))
    end++;
  if (*end != '\0')
    return false;
  *out = v;
  return true;
}

static bool deadband_exceeded(double last, double v, double del) {
  const bool ln = std::isnan(last), vn = std::isnan(v);
  if (ln || vn)
    return ln != vn;
  if (last == v)
    return false;
  const double d = std::fabs(v - last);
  if (std::isnan(d))
    return true;
  return d > del;
}

/// Format an IPv4 address given in host byte order.
static const char *format_ip4(char *buf, size_t len, uint32_t addr) {
  snprintf(buf, len, "%u.%u.%u.%u", unsigned((addr >> 24) & 0xFF), unsigned((addr >> 16) & 0xFF),
           unsigned((addr >> 8) & 0xFF), unsigned(addr & 0xFF));
  return buf;
}

// Big-endian byte writer
class ByteWriter {
 public:
  explicit ByteWriter(std::vector<uint8_t> &out) : out_(out) {}
  void u8(uint8_t v) { this->out_.push_back(v); }
  void u16(uint16_t v) {
    this->out_.push_back(static_cast<uint8_t>(v >> 8));
    this->out_.push_back(static_cast<uint8_t>(v));
  }
  void u32(uint32_t v) {
    this->u16(static_cast<uint16_t>(v >> 16));
    this->u16(static_cast<uint16_t>(v));
  }
  void i16(int16_t v) { this->u16(static_cast<uint16_t>(v)); }
  void i32(int32_t v) { this->u32(static_cast<uint32_t>(v)); }
  void f32(float v) {
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    this->u32(u);
  }
  void f64(double v) {
    uint64_t u;
    memcpy(&u, &v, sizeof(u));
    this->u32(static_cast<uint32_t>(u >> 32));
    this->u32(static_cast<uint32_t>(u));
  }
  /// Fixed-size, NUL terminated and NUL padded string field.
  void str(const char *s, size_t field) {
    const size_t n = strnlen(s, field - 1);
    this->out_.insert(this->out_.end(), s, s + n);
    this->out_.insert(this->out_.end(), field - n, uint8_t(0));
  }
  void zeros(size_t n) { this->out_.insert(this->out_.end(), n, uint8_t(0)); }

 private:
  std::vector<uint8_t> &out_;
};

static void put_header(std::vector<uint8_t> &v, uint16_t cmd, uint16_t psize, uint16_t dt, uint16_t count, uint32_t p1,
                       uint32_t p2) {
  ByteWriter w(v);
  w.u16(cmd);
  w.u16(psize);
  w.u16(dt);
  w.u16(count);
  w.u32(p1);
  w.u32(p2);
}

/// Header + payload padded to a multiple of 8 bytes (payloads here are always < 0xFFFF).
static void put_message(std::vector<uint8_t> &v, uint16_t cmd, uint16_t dt, uint16_t count, uint32_t p1, uint32_t p2,
                        const std::vector<uint8_t> &payload) {
  const size_t padded = padded_payload_size(payload.size());
  put_header(v, cmd, static_cast<uint16_t>(padded), dt, count, p1, p2);
  v.insert(v.end(), payload.begin(), payload.end());
  v.insert(v.end(), padded - payload.size(), uint8_t(0));
}

static bool is_valid_read_type(uint16_t t) {
  return t <= DBR_CTRL_DOUBLE || t == DBR_STSACK_STRING || t == DBR_CLASS_NAME;
}

static double data_as_number(const PVData &d) {
  if (d.type == NativeType::STRING) {
    double v;
    return parse_double(d.str, &v) ? v : 0.0;
  }
  return d.num;
}

static const char *data_as_string(char *buf, const PVData &d) {
  return native_to_string(buf, d.type, d.num, d.str, d.precision, d.enum_strs);
}

static void put_limits(ByteWriter &w, const PVData &d, uint16_t base, bool ctrl) {
  const double lim[DBR_CTRL_LIMIT_COUNT] = {d.upper_disp,    d.lower_disp,  d.upper_alarm, d.upper_warning,
                                            d.lower_warning, d.lower_alarm, d.upper_ctrl,  d.lower_ctrl};
  const int n = ctrl ? DBR_CTRL_LIMIT_COUNT : DBR_GR_LIMIT_COUNT;
  for (int i = 0; i < n; i++) {
    switch (base) {
      case DBR_SHORT:
        w.i16(saturate<int16_t>(lim[i]));
        break;
      case DBR_FLOAT:
        w.f32(to_f32(lim[i]));
        break;
      case DBR_CHAR:
        w.u8(saturate<uint8_t>(lim[i]));
        break;
      case DBR_LONG:
        w.i32(saturate<int32_t>(lim[i]));
        break;
      case DBR_DOUBLE:
        w.f64(lim[i]);
        break;
      default:
        break;
    }
  }
}

/// Encode one element (count == 1) of the given DBR type, laid out exactly like the
/// C structures in db_access.h.
static void encode_dbr(std::vector<uint8_t> &v, uint16_t dbr, const PVData &d) {
  ByteWriter w(v);
  char buf[FORMAT_BUFFER_SIZE];
  if (dbr == DBR_STSACK_STRING) {
    w.i16(d.status);
    w.i16(d.severity);
    w.u16(0);  // ackt
    w.u16(0);  // acks
    w.str(data_as_string(buf, d), MAX_STRING_SIZE);
    return;
  }
  if (dbr == DBR_CLASS_NAME) {
    w.str(d.class_name, MAX_STRING_SIZE);
    return;
  }
  const uint16_t base = dbr % DBR_TYPES_PER_CLASS;
  const uint16_t cls = dbr / DBR_TYPES_PER_CLASS;  // 0 plain, 1 STS, 2 TIME, 3 GR, 4 CTRL
  const bool ctrl = cls == DBR_CLASS_CTRL;
  if (cls >= DBR_CLASS_STS) {
    w.i16(d.status);
    w.i16(d.severity);
  }
  if (cls == DBR_CLASS_TIME) {
    w.u32(d.sec);
    w.u32(d.nsec);
  }
  const double num = data_as_number(d);
  switch (base) {
    case DBR_STRING:
      w.str(data_as_string(buf, d), MAX_STRING_SIZE);
      return;
    case DBR_SHORT:
      if (cls == DBR_CLASS_TIME)
        w.i16(0);  // RISC_pad
      if (cls >= DBR_CLASS_GR) {
        w.str(d.units, MAX_UNITS_SIZE);
        put_limits(w, d, base, ctrl);
      }
      w.i16(saturate<int16_t>(num));
      return;
    case DBR_FLOAT:
      if (cls >= DBR_CLASS_GR) {
        w.i16(d.precision);
        w.i16(0);  // RISC_pad0
        w.str(d.units, MAX_UNITS_SIZE);
        put_limits(w, d, base, ctrl);
      }
      w.f32(to_f32(num));
      return;
    case DBR_ENUM:
      if (cls == DBR_CLASS_TIME)
        w.i16(0);  // RISC_pad
      if (cls >= DBR_CLASS_GR) {
        const size_t no_str = std::min(d.enum_strs.size(), MAX_ENUM_STATES);
        w.i16(static_cast<int16_t>(no_str));
        for (size_t i = 0; i < MAX_ENUM_STATES; i++) {
          if (i < no_str) {
            w.str(d.enum_strs[i], MAX_ENUM_STRING_SIZE);
          } else {
            w.zeros(MAX_ENUM_STRING_SIZE);
          }
        }
      }
      w.u16(saturate<uint16_t>(num));
      return;
    case DBR_CHAR:
      if (cls == DBR_CLASS_STS) {
        w.u8(0);  // RISC_pad
      } else if (cls == DBR_CLASS_TIME) {
        w.i16(0);  // RISC_pad0
        w.u8(0);   // RISC_pad1
      } else if (cls >= DBR_CLASS_GR) {
        w.str(d.units, MAX_UNITS_SIZE);
        put_limits(w, d, base, ctrl);
        w.u8(0);  // RISC_pad
      }
      w.u8(saturate<uint8_t>(num));
      return;
    case DBR_LONG:
      if (cls >= DBR_CLASS_GR) {
        w.str(d.units, MAX_UNITS_SIZE);
        put_limits(w, d, base, ctrl);
      }
      w.i32(saturate<int32_t>(num));
      return;
    case DBR_DOUBLE:
      if (cls == DBR_CLASS_STS || cls == DBR_CLASS_TIME)
        w.i32(0);  // RISC_pad
      if (cls >= DBR_CLASS_GR) {
        w.i16(d.precision);
        w.i16(0);  // RISC_pad0
        w.str(d.units, MAX_UNITS_SIZE);
        put_limits(w, d, base, ctrl);
      }
      w.f64(num);
      return;
    default:
      return;
  }
}

/// Decode the first element of a put request.
static bool decode_put(uint16_t dt, uint32_t count, const uint8_t *p, size_t len, bool char_as_string, double *num,
                       std::string *str, bool *is_string, bool *integral) {
  *is_string = false;
  *integral = false;
  *num = NAN;
  str->clear();
  switch (dt) {
    case DBR_STRING: {
      if (len == 0)
        return false;
      const size_t n = std::min(len, MAX_STRING_SIZE);
      str->assign(reinterpret_cast<const char *>(p), strnlen(reinterpret_cast<const char *>(p), n));
      *is_string = true;
      return true;
    }
    case DBR_SHORT:
      if (len < 2)
        return false;
      *num = static_cast<int16_t>(rd16(p));
      *integral = true;
      return true;
    case DBR_FLOAT: {
      if (len < 4)
        return false;
      const uint32_t u = rd32(p);
      float f;
      memcpy(&f, &u, sizeof(f));
      *num = f;
      return true;
    }
    case DBR_ENUM:
      if (len < 2)
        return false;
      *num = rd16(p);
      *integral = true;
      return true;
    case DBR_CHAR:
      if (len < 1)
        return false;
      if (char_as_string && count > 1) {
        const size_t n = std::min(len, static_cast<size_t>(count));
        str->assign(reinterpret_cast<const char *>(p), strnlen(reinterpret_cast<const char *>(p), n));
        *is_string = true;
        return true;
      }
      *num = p[0];
      *integral = true;
      return true;
    case DBR_LONG:
      if (len < 4)
        return false;
      *num = static_cast<int32_t>(rd32(p));
      *integral = true;
      return true;
    case DBR_DOUBLE: {
      if (len < 8)
        return false;
      const uint64_t u = (static_cast<uint64_t>(rd32(p)) << 32) | rd32(p + 4);
      double dv;
      memcpy(&dv, &u, sizeof(dv));
      *num = dv;
      return true;
    }
    default:
      return false;
  }
}

static bool set_nonblocking(int fd) {
  int on = 1;
  return lwip_ioctl(fd, FIONBIO, &on) == 0;
}

static bool would_block(int err) { return err == EAGAIN || err == EWOULDBLOCK || err == EINTR; }

// ---------------------------------------------------------------------------
// EpicsPV
// ---------------------------------------------------------------------------
const char *EpicsPV::get_record_type() const {
  if (!this->record_type_.empty())
    return this->record_type_.c_str();
  const bool out = this->writable_;
  switch (this->type_) {
    case NativeType::STRING:
      return out ? "stringout" : "stringin";
    case NativeType::ENUM:
      if (this->enum_strs_.size() <= 2)
        return out ? "bo" : "bi";
      return out ? "mbbo" : "mbbi";
    case NativeType::FLOAT:
    case NativeType::DOUBLE:
      return out ? "ao" : "ai";
    default:
      return out ? "longout" : "longin";
  }
}

bool EpicsPV::is_integer_type_() const {
  return this->type_ == NativeType::SHORT || this->type_ == NativeType::CHAR || this->type_ == NativeType::LONG ||
         this->type_ == NativeType::ENUM;
}

void EpicsPV::init_(EpicsCAServer *server, const std::string &prefix) {
  this->server_ = server;
  this->full_name_ = prefix + this->name_;
  if (this->is_integer_type_())
    this->value_ = 0.0;
  epics_now(&this->sec_, &this->nsec_);
  this->setup_bindings_();
  if (this->initial_mode_ == InitialMode::NUMBER) {
    this->set_value(this->initial_num_);
  } else if (this->initial_mode_ == InitialMode::STRING) {
    this->set_string_value(this->initial_str_);
  }
}

void EpicsPV::setup_bindings_() {
#ifdef USE_SENSOR
  if (this->sensor_ != nullptr) {
    sensor::Sensor *s = this->sensor_;
    s->add_on_state_callback([this](float x) { this->set_value(x); });
    if (s->has_state())
      this->set_value(s->state);
  }
#endif
#ifdef USE_BINARY_SENSOR
  if (this->binary_sensor_ != nullptr) {
    binary_sensor::BinarySensor *bs = this->binary_sensor_;
    bs->add_on_state_callback([this](bool x) { this->set_value(x ? 1.0 : 0.0); });
    if (bs->has_state())
      this->set_value(bs->state ? 1.0 : 0.0);
  }
#endif
#ifdef USE_TEXT_SENSOR
  if (this->text_sensor_ != nullptr) {
    text_sensor::TextSensor *ts = this->text_sensor_;
    ts->add_on_state_callback([this](const std::string &x) { this->set_string_value(x); });
    if (ts->has_state())
      this->set_string_value(ts->state);
  }
#endif
#ifdef USE_NUMBER
  if (this->number_ != nullptr) {
    number::Number *n = this->number_;
    const double min_v = n->traits.get_min_value();
    const double max_v = n->traits.get_max_value();
    if (std::isnan(this->hopr_) && std::isnan(this->lopr_) && !std::isnan(min_v) && !std::isnan(max_v)) {
      this->hopr_ = max_v;
      this->lopr_ = min_v;
    }
    if (std::isnan(this->drvh_) && !std::isnan(max_v))
      this->drvh_ = max_v;
    if (std::isnan(this->drvl_) && !std::isnan(min_v))
      this->drvl_ = min_v;
    n->add_on_state_callback([this](float x) { this->set_value(x); });
    this->write_handler_ = [n](double v, const std::string &) {
      if (std::isnan(v))
        return false;
      auto call = n->make_call();
      call.set_value(static_cast<float>(v));
      call.perform();
      return true;
    };
    if (n->has_state())
      this->set_value(n->state);
  }
#endif
#ifdef USE_SWITCH
  if (this->switch_entity_ != nullptr) {
    switch_::Switch *sw = this->switch_entity_;
    sw->add_on_state_callback([this](bool x) { this->set_value(x ? 1.0 : 0.0); });
    this->write_handler_ = [sw](double v, const std::string &) {
      if (v != 0.0) {
        sw->turn_on();
      } else {
        sw->turn_off();
      }
      return true;
    };
    this->set_value(sw->state ? 1.0 : 0.0);
  }
#endif
#ifdef USE_SELECT
  if (this->select_ != nullptr) {
    select::Select *sel = this->select_;
    if (this->enum_strs_.empty()) {
      for (const auto &opt : sel->traits.get_options()) {
        std::string s(opt);
        if (s.size() > MAX_ENUM_STRING_SIZE - 1)
          s.resize(MAX_ENUM_STRING_SIZE - 1);
        this->enum_strs_.push_back(s);
      }
      if (this->enum_strs_.size() > MAX_ENUM_STATES) {
        ESP_LOGW(TAG, "PV %s: select has %u options, only the first %u are reported in DBR_GR/CTRL_ENUM",
                 this->full_name_.c_str(), unsigned(this->enum_strs_.size()), unsigned(MAX_ENUM_STATES));
      }
    }
    sel->add_on_state_callback([this, sel](auto &&...) {
      auto idx = sel->active_index();
      if (idx.has_value()) {
        this->set_value(static_cast<double>(*idx));
      } else {
        this->set_undefined();
      }
    });
    this->write_handler_ = [sel](double v, const std::string &) {
      if (std::isnan(v) || v < 0)
        return false;
      auto call = sel->make_call();
      call.set_index(static_cast<size_t>(v));
      call.perform();
      return true;
    };
    auto idx = sel->active_index();
    if (idx.has_value())
      this->set_value(static_cast<double>(*idx));
  }
#endif
#ifdef USE_TEXT
  if (this->text_entity_ != nullptr) {
    text::Text *t = this->text_entity_;
    t->add_on_state_callback([this](const std::string &x) { this->set_string_value(x); });
    this->write_handler_ = [t](double, const std::string &s) {
      auto call = t->make_call();
      call.set_value(s);
      call.perform();
      return true;
    };
    if (t->has_state())
      this->set_string_value(t->state);
  }
#endif
#ifdef USE_BUTTON
  if (this->button_ != nullptr) {
    button::Button *b = this->button_;
    this->write_handler_ = [b](double v, const std::string &) {
      if (!std::isnan(v) && v != 0.0)
        b->press();
      return true;
    };
    this->set_value(0.0);
  }
#endif
}

void EpicsPV::set_value(double value) {
  if (this->type_ == NativeType::STRING) {
    if (std::isnan(value)) {
      this->apply_string_(this->str_value_, false);
    } else {
      this->apply_string_(this->number_to_string_(value), true);
    }
    return;
  }
  this->apply_number_(value);
}

void EpicsPV::set_string_value(const std::string &value) {
  if (this->type_ == NativeType::STRING) {
    this->apply_string_(value, true);
    return;
  }
  double v;
  if (this->parse_number_(value, &v)) {
    this->apply_number_(v);
  } else {
    this->apply_number_(NAN);
  }
}

void EpicsPV::set_undefined() {
  if (this->type_ == NativeType::STRING) {
    this->apply_string_(this->str_value_, false);
  } else {
    this->apply_number_(NAN);
  }
}

bool EpicsPV::parse_number_(const std::string &text, double *out) const {
  if (this->type_ == NativeType::ENUM) {
    for (size_t i = 0; i < this->enum_strs_.size(); i++) {
      if (this->enum_strs_[i] == text) {
        *out = static_cast<double>(i);
        return true;
      }
    }
  }
  return parse_double(text.c_str(), out);
}

std::string EpicsPV::number_to_string_(double value) const {
  char buf[FORMAT_BUFFER_SIZE];
  if (this->type_ == NativeType::STRING)
    return format_double(buf, value, this->precision_);
  return native_to_string(buf, this->type_, value, this->str_value_.c_str(), this->precision_,
                          EnumStrings(this->enum_strs_));
}

void EpicsPV::compute_limit_alarm_(double value, int16_t *status, int16_t *severity) {
  const double h = this->hyst_;
  const uint8_t last = this->alarm_level_;
  uint8_t lvl = LEVEL_NONE;
  if (this->hihi_sev_ > 0 && !std::isnan(this->hihi_) &&
      (value >= this->hihi_ || (last == LEVEL_HIHI && value >= this->hihi_ - h))) {
    lvl = LEVEL_HIHI;
  } else if (this->lolo_sev_ > 0 && !std::isnan(this->lolo_) &&
             (value <= this->lolo_ || (last == LEVEL_LOLO && value <= this->lolo_ + h))) {
    lvl = LEVEL_LOLO;
  } else if (this->high_sev_ > 0 && !std::isnan(this->high_) &&
             (value >= this->high_ || (last == LEVEL_HIGH && value >= this->high_ - h))) {
    lvl = LEVEL_HIGH;
  } else if (this->low_sev_ > 0 && !std::isnan(this->low_) &&
             (value <= this->low_ || (last == LEVEL_LOW && value <= this->low_ + h))) {
    lvl = LEVEL_LOW;
  }
  this->alarm_level_ = lvl;
  switch (lvl) {
    case LEVEL_HIHI:
      *status = STAT_HIHI;
      *severity = this->hihi_sev_;
      break;
    case LEVEL_LOLO:
      *status = STAT_LOLO;
      *severity = this->lolo_sev_;
      break;
    case LEVEL_HIGH:
      *status = STAT_HIGH;
      *severity = this->high_sev_;
      break;
    case LEVEL_LOW:
      *status = STAT_LOW;
      *severity = this->low_sev_;
      break;
    default:
      *status = STAT_NO_ALARM;
      *severity = SEV_NO_ALARM;
      break;
  }
}

void EpicsPV::apply_number_(double value) {
  double v = value;
  const bool defined = !std::isnan(v);
  if (defined) {
    switch (this->type_) {
      case NativeType::SHORT:
        v = saturate<int16_t>(std::round(v));
        break;
      case NativeType::CHAR:
        v = saturate<uint8_t>(std::round(v));
        break;
      case NativeType::LONG:
        v = saturate<int32_t>(std::round(v));
        break;
      case NativeType::ENUM:
        v = saturate<uint16_t>(std::round(v));
        break;
      case NativeType::FLOAT:
        v = static_cast<double>(to_f32(v));
        break;
      default:
        break;
    }
  }
  // Integer types keep their last value when undefined; floating types carry the NaN.
  if (defined || is_floating(this->type_))
    this->value_ = v;
  this->defined_ = defined;

  int16_t stat = STAT_NO_ALARM;
  int16_t sev = SEV_NO_ALARM;
  if (!defined) {
    stat = STAT_UDF;
    sev = SEV_INVALID;
    this->alarm_level_ = LEVEL_NONE;
  } else if (this->type_ == NativeType::ENUM) {
    const size_t idx = saturate<uint16_t>(v);
    if (idx < this->enum_sevs_.size() && this->enum_sevs_[idx] > 0) {
      stat = STAT_STATE;
      sev = this->enum_sevs_[idx];
    }
  } else {
    this->compute_limit_alarm_(v, &stat, &sev);
  }

  epics_now(&this->sec_, &this->nsec_);

  uint16_t mask = 0;
  if (this->mdel_ < 0 || deadband_exceeded(this->mlst_, this->value_, this->mdel_)) {
    mask |= DBE_VALUE;
    this->mlst_ = this->value_;
  }
  if (this->adel_ < 0 || deadband_exceeded(this->alst_, this->value_, this->adel_)) {
    mask |= DBE_LOG;
    this->alst_ = this->value_;
  }
  this->update_alarm_and_post_(stat, sev, mask);
}

void EpicsPV::apply_string_(const std::string &value, bool defined) {
  const size_t len = std::min(value.size(), MAX_STRING_CHARS);
  const bool changed = this->str_value_.compare(0, std::string::npos, value, 0, len) != 0 || defined != this->defined_;
  // `value` is str_value_ itself when the current string only becomes undefined.
  if (&value != &this->str_value_)
    this->str_value_.assign(value, 0, len);
  this->defined_ = defined;
  epics_now(&this->sec_, &this->nsec_);
  const uint16_t mask = (changed || this->mdel_ < 0) ? (DBE_VALUE | DBE_LOG) : 0;
  this->update_alarm_and_post_(defined ? STAT_NO_ALARM : STAT_UDF, defined ? SEV_NO_ALARM : SEV_INVALID, mask);
}

void EpicsPV::update_alarm_and_post_(int16_t status, int16_t severity, uint16_t value_mask) {
  uint16_t mask = value_mask;
  const bool alarm_changed = status != this->status_ || severity != this->severity_;
  if (alarm_changed) {
    this->status_ = status;
    this->severity_ = severity;
    mask |= DBE_ALARM;
  }
  if (this->server_ == nullptr)
    return;
  if (mask != 0)
    this->server_->post_event(this, PVField::FIELD_VAL, mask);
  if (alarm_changed) {
    this->server_->post_event(this, PVField::FIELD_SEVR, DBE_VALUE | DBE_LOG | DBE_ALARM);
    this->server_->post_event(this, PVField::FIELD_STAT, DBE_VALUE | DBE_LOG | DBE_ALARM);
  }
}

uint32_t EpicsPV::client_write_(double num, const std::string &str, bool is_string, bool integral) {
  if (!this->writable_)
    return ECA_NOWTACCESS;

  double v = NAN;
  std::string s;
  if (this->type_ == NativeType::STRING) {
    char buf[FORMAT_BUFFER_SIZE];
    if (is_string) {
      s = str;
    } else if (integral) {
      s = format_integer(buf, num);
    } else {
      s = format_double(buf, num, this->precision_);
    }
    if (s.size() > MAX_STRING_CHARS)
      s.resize(MAX_STRING_CHARS);
    double parsed;
    if (parse_double(s.c_str(), &parsed))
      v = parsed;
  } else {
    if (is_string) {
      if (!this->parse_number_(str, &v))
        return ECA_PUTFAIL;
    } else {
      v = num;
    }
    if (std::isnan(v)) {
      if (!is_floating(this->type_))
        return ECA_PUTFAIL;
    } else {
      if (!std::isnan(this->drvh_) && !std::isnan(this->drvl_) && this->drvh_ > this->drvl_) {
        if (v > this->drvh_)
          v = this->drvh_;
        if (v < this->drvl_)
          v = this->drvl_;
      } else if (!std::isnan(this->drvh_) && std::isnan(this->drvl_) && v > this->drvh_) {
        v = this->drvh_;
      } else if (std::isnan(this->drvh_) && !std::isnan(this->drvl_) && v < this->drvl_) {
        v = this->drvl_;
      }
      switch (this->type_) {
        case NativeType::SHORT:
          v = saturate<int16_t>(v);
          break;
        case NativeType::CHAR:
          v = saturate<uint8_t>(v);
          break;
        case NativeType::LONG:
          v = saturate<int32_t>(v);
          break;
        case NativeType::ENUM:
          if (v < 0)
            return ECA_PUTFAIL;
          v = saturate<uint16_t>(v);
          if (!this->enum_strs_.empty() && v >= static_cast<double>(this->enum_strs_.size()))
            return ECA_PUTFAIL;
          break;
        case NativeType::FLOAT:
          v = static_cast<double>(to_f32(v));
          break;
        default:
          break;
      }
    }
    s = this->number_to_string_(v);
  }

  if (this->write_handler_) {
    if (!this->write_handler_(v, s))
      return ECA_PUTFAIL;
  } else if (this->type_ == NativeType::STRING) {
    this->apply_string_(s, true);
  } else {
    this->apply_number_(v);
  }
  this->write_callback_.call(v, s);
  return ECA_NORMAL;
}

NativeType EpicsPV::field_native_type_(PVField field) const {
  switch (field) {
    case PVField::FIELD_VAL:
      return this->type_;
    case PVField::FIELD_NAME:
    case PVField::FIELD_DESC:
    case PVField::FIELD_EGU:
    case PVField::FIELD_RTYP:
      return NativeType::STRING;
    case PVField::FIELD_PREC:
      return NativeType::SHORT;
    case PVField::FIELD_SEVR:
    case PVField::FIELD_STAT:
      return NativeType::ENUM;
    default:
      return NativeType::DOUBLE;
  }
}

void EpicsPV::fill_data_(PVField field, PVData &d) const {
  d = PVData();
  d.type = this->field_native_type_(field);
  d.status = this->status_;
  d.severity = this->severity_;
  d.sec = this->sec_;
  d.nsec = this->nsec_;
  d.class_name = this->get_record_type();

  auto numeric_field = [&](double v) {
    d.num = std::isnan(v) ? 0.0 : v;
    d.units = this->units_.c_str();
    d.precision = this->precision_;
  };

  switch (field) {
    case PVField::FIELD_VAL: {
      d.num = this->value_;
      d.str = this->str_value_.c_str();
      d.enum_strs = EnumStrings(this->enum_strs_);
      d.precision = this->precision_;
      if (this->type_ != NativeType::STRING && this->type_ != NativeType::ENUM) {
        d.units = this->units_.c_str();
        const double hopr = std::isnan(this->hopr_) ? 0.0 : this->hopr_;
        const double lopr = std::isnan(this->lopr_) ? 0.0 : this->lopr_;
        d.upper_disp = hopr;
        d.lower_disp = lopr;
        d.upper_alarm = this->hihi_sev_ > 0 ? this->hihi_ : NAN;
        d.upper_warning = this->high_sev_ > 0 ? this->high_ : NAN;
        d.lower_warning = this->low_sev_ > 0 ? this->low_ : NAN;
        d.lower_alarm = this->lolo_sev_ > 0 ? this->lolo_ : NAN;
        d.upper_ctrl = std::isnan(this->drvh_) ? hopr : this->drvh_;
        d.lower_ctrl = std::isnan(this->drvl_) ? lopr : this->drvl_;
      }
      break;
    }
    case PVField::FIELD_NAME:
      d.str = this->full_name_.c_str();
      break;
    case PVField::FIELD_DESC:
      d.str = this->description_.c_str();
      break;
    case PVField::FIELD_EGU:
      d.str = this->units_.c_str();
      break;
    case PVField::FIELD_RTYP:
      d.str = d.class_name;
      break;
    case PVField::FIELD_PREC:
      d.num = this->precision_;
      break;
    case PVField::FIELD_HOPR:
      numeric_field(this->hopr_);
      break;
    case PVField::FIELD_LOPR:
      numeric_field(this->lopr_);
      break;
    case PVField::FIELD_DRVH:
      numeric_field(this->drvh_);
      break;
    case PVField::FIELD_DRVL:
      numeric_field(this->drvl_);
      break;
    case PVField::FIELD_HIHI:
      numeric_field(this->hihi_);
      break;
    case PVField::FIELD_HIGH:
      numeric_field(this->high_);
      break;
    case PVField::FIELD_LOW:
      numeric_field(this->low_);
      break;
    case PVField::FIELD_LOLO:
      numeric_field(this->lolo_);
      break;
    case PVField::FIELD_HYST:
      numeric_field(this->hyst_);
      break;
    case PVField::FIELD_MDEL:
      numeric_field(this->mdel_);
      break;
    case PVField::FIELD_ADEL:
      numeric_field(this->adel_);
      break;
    case PVField::FIELD_SEVR:
      d.num = this->severity_;
      d.enum_strs = EnumStrings(SEVERITY_STRINGS, std::size(SEVERITY_STRINGS));
      break;
    case PVField::FIELD_STAT:
      d.num = this->status_;
      d.enum_strs = EnumStrings(STATUS_STRINGS, std::size(STATUS_STRINGS));
      break;
  }
}

// ---------------------------------------------------------------------------
// EpicsCAServer
// ---------------------------------------------------------------------------
void EpicsCAServer::setup() {
  for (auto *pv : this->pvs_)
    pv->init_(this, this->prefix_);
}

void EpicsCAServer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "EPICS Channel Access server:\n"
                "  Protocol: CA v4.%u\n"
                "  Port: %u\n"
                "  Prefix: '%s'\n"
                "  Beacons: %s\n"
                "  Beacon port: %u\n"
                "  Beacon period: %" PRIu32 " ms",
                unsigned(CA_MINOR_PROTOCOL_REVISION), unsigned(this->port_), this->prefix_.c_str(),
                YESNO(this->beacons_enabled_), unsigned(this->beacon_port_), this->beacon_period_ms_);
  for (uint32_t addr : this->beacon_addrs_) {
    char ip[IP4_STR_SIZE];
    ESP_LOGCONFIG(TAG, "  Beacon address: %s", format_ip4(ip, sizeof(ip), addr));
  }
  ESP_LOGCONFIG(TAG,
                "  Max clients: %u\n"
                "  Max channels per client: %u\n"
                "  Max subscriptions per client: %u\n"
                "  Send buffer size: %u bytes\n"
                "  Max payload size: %u bytes\n"
                "  Read only: %s\n"
                "  Field access: %s\n"
                "  TCP keepalive: %s",
                unsigned(this->max_clients_), unsigned(this->max_channels_), unsigned(this->max_subs_),
                unsigned(this->send_buffer_size_), unsigned(this->max_payload_), YESNO(this->read_only_),
                YESNO(this->field_access_), YESNO(this->tcp_keepalive_));
  for (auto *pv : this->pvs_) {
    ESP_LOGCONFIG(
        TAG, "  PV '%s' (DBR type %u, %s, %s)", pv->get_full_name().c_str(), unsigned(pv->get_type()),
        pv->get_record_type(),
        this->can_write_(pv, PVField::FIELD_VAL) ? LOG_STR_LITERAL("read/write") : LOG_STR_LITERAL("read-only"));
  }
}

bool EpicsCAServer::start_(uint32_t now) {
  const int one = 1;
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(this->port_);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  const int udp = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp < 0) {
    ESP_LOGW(TAG, "Could not create UDP socket (errno %d)", errno);
    return false;
  }
  lwip_setsockopt(udp, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  lwip_setsockopt(udp, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  if (lwip_bind(udp, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0 || !set_nonblocking(udp)) {
    ESP_LOGW(TAG, "Could not bind UDP port %u (errno %d)", unsigned(this->port_), errno);
    lwip_close(udp);
    return false;
  }

  const int tcp = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (tcp < 0) {
    ESP_LOGW(TAG, "Could not create TCP socket (errno %d)", errno);
    lwip_close(udp);
    return false;
  }
  lwip_setsockopt(tcp, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if (lwip_bind(tcp, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0 ||
      lwip_listen(tcp, static_cast<int>(this->max_clients_)) != 0 || !set_nonblocking(tcp)) {
    ESP_LOGW(TAG, "Could not listen on TCP port %u (errno %d)", unsigned(this->port_), errno);
    lwip_close(tcp);
    lwip_close(udp);
    return false;
  }

  this->udp_fd_ = udp;
  this->tcp_fd_ = tcp;
  this->started_ = true;
  this->beacon_interval_ms_ = BEACON_INITIAL_INTERVAL_MS;
  this->last_beacon_ms_ = now - this->beacon_interval_ms_;
  ESP_LOGI(TAG, "Listening on port %u with %u PVs", unsigned(this->port_), unsigned(this->pvs_.size()));
  return true;
}

void EpicsCAServer::stop_() {
  for (auto &c : this->clients_) {
    if (c->fd >= 0)
      lwip_close(c->fd);
  }
  this->clients_.clear();
  if (this->tcp_fd_ >= 0)
    lwip_close(this->tcp_fd_);
  if (this->udp_fd_ >= 0)
    lwip_close(this->udp_fd_);
  this->tcp_fd_ = -1;
  this->udp_fd_ = -1;
  this->started_ = false;
}

void EpicsCAServer::on_shutdown() { this->stop_(); }

void EpicsCAServer::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  if (!this->started_) {
    if (!network::is_connected())
      return;
    if (this->start_attempted_ && now - this->last_start_attempt_ < START_RETRY_INTERVAL_MS)
      return;
    this->start_attempted_ = true;
    this->last_start_attempt_ = now;
    if (!this->start_(now))
      return;
  }

  this->handle_udp_();
  this->accept_clients_();

  for (auto &cp : this->clients_) {
    CAClient &c = *cp;
    if (!c.closing)
      this->read_client_(c);
    if (!c.closing)
      this->service_pending_(c);
    if (!c.closing)
      this->flush_(c);
  }

  auto it = std::remove_if(this->clients_.begin(), this->clients_.end(), [](const std::unique_ptr<CAClient> &c) {
    if (!c->closing)
      return false;
    ESP_LOGD(TAG, "Client %s disconnected", c->peer);
    if (c->fd >= 0)
      lwip_close(c->fd);
    return true;
  });
  this->clients_.erase(it, this->clients_.end());

  this->send_beacons_(now);
}

// ---- name lookup ----
EpicsPV *EpicsCAServer::find_pv_(const std::string &name, PVField *field) const {
  size_t base_len = name.size();
  PVField f = PVField::FIELD_VAL;
  const size_t dot = name.find('.');
  if (dot != std::string::npos) {
    const char *fname = name.c_str() + dot + 1;
    base_len = dot;
    if (strcmp(fname, "VAL") != 0 && !this->field_access_)
      return nullptr;
    bool found = false;
    for (const auto &fn : FIELD_NAMES) {
      if (strcmp(fname, fn.name) == 0) {
        f = fn.field;
        found = true;
        break;
      }
    }
    if (!found)
      return nullptr;
  }
  for (auto *pv : this->pvs_) {
    if (pv->full_name_.size() == base_len && pv->full_name_.compare(0, base_len, name, 0, base_len) == 0) {
      *field = f;
      return pv;
    }
  }
  return nullptr;
}

bool EpicsCAServer::can_write_(const EpicsPV *pv, PVField field) const {
  return !this->read_only_ && field == PVField::FIELD_VAL && pv->writable_;
}

// ---- UDP: name resolution ----
void EpicsCAServer::handle_udp_() {
  uint8_t buf[UDP_RECV_BUFFER];
  for (int i = 0; i < MAX_DATAGRAMS_PER_LOOP; i++) {
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    const ssize_t n =
        lwip_recvfrom(this->udp_fd_, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr *>(&from), &fromlen);
    if (n < 0)
      break;
    if (n == 0 || from.sin_family != AF_INET)
      continue;
    this->process_datagram_(buf, static_cast<size_t>(n), &from);
  }
}

void EpicsCAServer::process_datagram_(const uint8_t *buf, size_t len, const void *from_addr) {
  const struct sockaddr_in *from = static_cast<const struct sockaddr_in *>(from_addr);
  std::vector<uint8_t> &reply = this->udp_buf_;
  reply.clear();
  uint16_t version_dt = 0;
  uint32_t sequence = 0;

  auto send_reply = [&]() {
    // A reply holding only the version header has nothing to say.
    if (reply.size() > CA_HEADER_SIZE) {
      lwip_sendto(this->udp_fd_, reply.data(), reply.size(), 0, reinterpret_cast<const struct sockaddr *>(from),
                  sizeof(struct sockaddr_in));
    }
    reply.clear();
  };
  auto ensure_room = [&](size_t needed) {
    if (!reply.empty() && reply.size() + needed > MAX_UDP_SEND)
      send_reply();
    if (reply.empty())
      put_header(reply, CA_PROTO_VERSION, 0, version_dt, CA_MINOR_PROTOCOL_REVISION, sequence, 0);
  };

  size_t off = 0;
  while (len - off >= CA_HEADER_SIZE) {
    const uint8_t *p = buf + off;
    const uint16_t cmd = rd16(p);
    uint32_t psize = rd16(p + 2);
    const uint16_t dt = rd16(p + 4);
    uint32_t count = rd16(p + 6);
    const uint32_t p1 = rd32(p + 8);
    const uint32_t p2 = rd32(p + 12);
    size_t hlen = CA_HEADER_SIZE;
    if (psize == CA_EXTENDED_POSTSIZE && count == 0) {
      if (len - off < CA_EXTENDED_HEADER_SIZE)
        break;
      psize = rd32(p + 16);
      count = rd32(p + 20);
      hlen = CA_EXTENDED_HEADER_SIZE;
    }
    if (psize > len - off - hlen)
      break;
    const uint8_t *payload = p + hlen;

    if (cmd == CA_PROTO_VERSION) {
      version_dt = dt;
      sequence = p1;
    } else if (cmd == CA_PROTO_SEARCH) {
      const size_t nlen = strnlen(reinterpret_cast<const char *>(payload), psize);
      const std::string name(reinterpret_cast<const char *>(payload), nlen);
      PVField field;
      if (this->find_pv_(name, &field) != nullptr) {
        ensure_room(CA_HEADER_SIZE + CA_SEARCH_REPLY_PAYLOAD_SIZE);
        // data_type = server TCP port, p1 = ~0 (client uses the reply's source address), p2 = search id
        put_header(reply, CA_PROTO_SEARCH, CA_SEARCH_REPLY_PAYLOAD_SIZE, this->port_, 0,
                   CA_SEARCH_REPLY_USE_SOURCE_ADDR, p2);
        ByteWriter w(reply);
        w.u16(CA_MINOR_PROTOCOL_REVISION);
        w.zeros(CA_SEARCH_REPLY_PAYLOAD_SIZE - sizeof(uint16_t));
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
        char ip[IP4_STR_SIZE];
        ESP_LOGV(TAG, "Search for '%s' from %s answered", name.c_str(),
                 format_ip4(ip, sizeof(ip), ntohl(from->sin_addr.s_addr)));
#endif
      } else if (dt == SEARCH_DO_REPLY) {
        ensure_room(CA_HEADER_SIZE);
        put_header(reply, CA_PROTO_NOT_FOUND, 0, dt, static_cast<uint16_t>(count), p1, p2);
      }
    }
    off += hlen + psize;
  }
  send_reply();
}

void EpicsCAServer::send_beacons_(uint32_t now) {
  if (!this->beacons_enabled_ || this->beacon_addrs_.empty())
    return;
  if (now - this->last_beacon_ms_ < this->beacon_interval_ms_)
    return;
  this->last_beacon_ms_ = now;

  std::vector<uint8_t> &msg = this->udp_buf_;
  msg.clear();
  // data_type = protocol minor version, count = server port, p1 = beacon id, p2 = INADDR_ANY (use source)
  put_header(msg, CA_PROTO_RSRV_IS_UP, 0, CA_MINOR_PROTOCOL_REVISION, this->port_, this->beacon_id_++, 0);
  for (uint32_t a : this->beacon_addrs_) {
    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(this->beacon_port_);
    to.sin_addr.s_addr = htonl(a);
    lwip_sendto(this->udp_fd_, msg.data(), msg.size(), 0, reinterpret_cast<struct sockaddr *>(&to), sizeof(to));
  }
  if (this->beacon_interval_ms_ < this->beacon_period_ms_) {
    this->beacon_interval_ms_ *= 2;
    if (this->beacon_interval_ms_ > this->beacon_period_ms_)
      this->beacon_interval_ms_ = this->beacon_period_ms_;
  }
}

// ---- TCP: virtual circuits ----
void EpicsCAServer::accept_clients_() {
  for (int i = 0; i < MAX_ACCEPTS_PER_LOOP; i++) {
    struct sockaddr_in a;
    socklen_t alen = sizeof(a);
    const int fd = lwip_accept(this->tcp_fd_, reinterpret_cast<struct sockaddr *>(&a), &alen);
    if (fd < 0)
      return;
    if (this->clients_.size() >= this->max_clients_) {
      char ip[IP4_STR_SIZE];
      ESP_LOGW(TAG, "Rejecting client %s: max_clients (%u) reached",
               format_ip4(ip, sizeof(ip), ntohl(a.sin_addr.s_addr)), unsigned(this->max_clients_));
      lwip_close(fd);
      continue;
    }
    if (!set_nonblocking(fd)) {
      lwip_close(fd);
      continue;
    }
    const int one = 1;
    lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    if (this->tcp_keepalive_) {
      lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
#if defined(TCP_KEEPIDLE) && defined(TCP_KEEPINTVL) && defined(TCP_KEEPCNT)
      const int idle = TCP_KEEPALIVE_IDLE_S, intvl = TCP_KEEPALIVE_INTERVAL_S, cnt = TCP_KEEPALIVE_COUNT;
      lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
      lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
      lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
    }
    auto c = make_unique<CAClient>();
    c->fd = fd;
    char ip[IP4_STR_SIZE];
    snprintf(c->peer, sizeof(c->peer), "%s:%u", format_ip4(ip, sizeof(ip), ntohl(a.sin_addr.s_addr)),
             unsigned(ntohs(a.sin_port)));
    // The server announces its protocol version as soon as the circuit is up.
    put_header(c->tx, CA_PROTO_VERSION, 0, 0, CA_MINOR_PROTOCOL_REVISION, 0, 0);
    ESP_LOGD(TAG, "Client %s connected", c->peer);
    this->clients_.push_back(std::move(c));
  }
}

void EpicsCAServer::read_client_(CAClient &c) {
  uint8_t buf[TCP_RECV_CHUNK];
  for (int i = 0; i < MAX_TCP_READS_PER_LOOP; i++) {
    const ssize_t n = lwip_recv(c.fd, buf, sizeof(buf), 0);
    if (n > 0) {
      c.rx.insert(c.rx.end(), buf, buf + n);
      continue;
    }
    if (n == 0) {
      c.closing = true;
      return;
    }
    if (would_block(errno))
      break;
    c.closing = true;
    return;
  }
  this->process_rx_(c);
}

void EpicsCAServer::process_rx_(CAClient &c) {
  size_t off = 0;
  while (!c.closing) {
    const size_t avail = c.rx.size() - off;
    if (avail < CA_HEADER_SIZE)
      break;
    const uint8_t *p = c.rx.data() + off;
    CAHeader h;
    h.cmd = rd16(p);
    h.psize = rd16(p + 2);
    h.dt = rd16(p + 4);
    h.count = rd16(p + 6);
    h.p1 = rd32(p + 8);
    h.p2 = rd32(p + 12);
    size_t hlen = CA_HEADER_SIZE;
    if (h.psize == CA_EXTENDED_POSTSIZE && h.count == 0) {
      if (avail < CA_EXTENDED_HEADER_SIZE)
        break;
      h.psize = rd32(p + 16);
      h.count = rd32(p + 20);
      hlen = CA_EXTENDED_HEADER_SIZE;
    }
    if (h.psize > this->max_payload_) {
      ESP_LOGW(TAG, "Client %s: message payload of %" PRIu32 " bytes exceeds max_payload_size, disconnecting", c.peer,
               h.psize);
      c.closing = true;
      break;
    }
    if (avail < hlen + h.psize)
      break;
    this->handle_message_(c, h, p, p + hlen);
    off += hlen + h.psize;
  }
  if (off > 0)
    c.rx.erase(c.rx.begin(), c.rx.begin() + off);
}

void EpicsCAServer::handle_message_(CAClient &c, const CAHeader &h, const uint8_t *raw, const uint8_t *payload) {
  switch (h.cmd) {
    case CA_PROTO_VERSION:
      c.priority = h.dt;
      c.minor = static_cast<uint16_t>(h.count);
      break;
    case CA_PROTO_ECHO:
      this->queue_header_(c, CA_PROTO_ECHO, 0, 0, 0, 0);
      break;
    case CA_PROTO_CLIENT_NAME:
      c.user.assign(
          reinterpret_cast<const char *>(payload),
          strnlen(reinterpret_cast<const char *>(payload), std::min<size_t>(h.psize, MAX_CLIENT_NAME_LENGTH)));
      break;
    case CA_PROTO_HOST_NAME:
      c.host.assign(
          reinterpret_cast<const char *>(payload),
          strnlen(reinterpret_cast<const char *>(payload), std::min<size_t>(h.psize, MAX_CLIENT_NAME_LENGTH)));
      break;
    case CA_PROTO_CREATE_CHAN:
      this->handle_create_chan_(c, h, payload);
      break;
    case CA_PROTO_CLEAR_CHANNEL:
      this->handle_clear_channel_(c, h, raw);
      break;
    case CA_PROTO_READ_NOTIFY:
      this->handle_read_notify_(c, h, raw);
      break;
    case CA_PROTO_WRITE:
    case CA_PROTO_WRITE_NOTIFY:
      this->handle_write_(c, h, raw, payload);
      break;
    case CA_PROTO_EVENT_ADD:
      this->handle_event_add_(c, h, raw, payload);
      break;
    case CA_PROTO_EVENT_CANCEL:
      this->handle_event_cancel_(c, h, raw);
      break;
    case CA_PROTO_EVENTS_OFF:
      c.events_off = true;
      break;
    case CA_PROTO_EVENTS_ON:
      c.events_off = false;
      for (auto &s : c.subs)
        s.pending = true;  // deliver the latest value of every subscription
      break;
    case CA_PROTO_READ_SYNC:
      this->queue_header_(c, CA_PROTO_READ_SYNC, 0, 0, 0, 0);
      break;
    default:
      ESP_LOGV(TAG, "Ignoring CA command %u", unsigned(h.cmd));
      break;
  }
}

CAChannel *EpicsCAServer::find_channel_(CAClient &c, uint32_t sid) {
  for (auto &ch : c.channels) {
    if (ch.sid == sid)
      return &ch;
  }
  return nullptr;
}

void EpicsCAServer::handle_create_chan_(CAClient &c, const CAHeader &h, const uint8_t *payload) {
  const std::string name(reinterpret_cast<const char *>(payload),
                         strnlen(reinterpret_cast<const char *>(payload), h.psize));
  PVField field = PVField::FIELD_VAL;
  EpicsPV *pv = this->find_pv_(name, &field);
  if (pv == nullptr || c.channels.size() >= this->max_channels_) {
    if (pv != nullptr) {
      ESP_LOGW(TAG, "Client %s: max_channels_per_client reached, refusing '%s'", c.peer, name.c_str());
    }
    this->queue_header_(c, CA_PROTO_CREATE_CH_FAIL, 0, 0, h.p1, 0);
    return;
  }
  const uint32_t sid = this->next_sid_++;
  c.channels.push_back(CAChannel{sid, h.p1, pv, field});
  const uint32_t rights = CA_ACCESS_READ | (this->can_write_(pv, field) ? CA_ACCESS_WRITE : 0);
  this->queue_header_(c, CA_PROTO_ACCESS_RIGHTS, 0, 0, h.p1, rights);
  this->queue_header_(c, CA_PROTO_CREATE_CHAN, static_cast<uint16_t>(pv->field_native_type_(field)), 1, h.p1, sid);
  ESP_LOGD(TAG, "Client %s (%s@%s): channel '%s' created (sid %" PRIu32 ")", c.peer, c.user.c_str(), c.host.c_str(),
           name.c_str(), sid);
}

void EpicsCAServer::handle_clear_channel_(CAClient &c, const CAHeader &h, const uint8_t *raw) {
  auto it = std::find_if(c.channels.begin(), c.channels.end(), [&](const CAChannel &ch) { return ch.sid == h.p1; });
  if (it == c.channels.end()) {
    this->send_error_(c, raw, h.p2, ECA_BADCHID, ERR_BAD_CHANNEL_ID);
    return;
  }
  c.subs.erase(std::remove_if(c.subs.begin(), c.subs.end(), [&](const CASubscription &s) { return s.sid == h.p1; }),
               c.subs.end());
  c.channels.erase(it);
  this->queue_header_(c, CA_PROTO_CLEAR_CHANNEL, 0, 0, h.p1, h.p2);
}

void EpicsCAServer::handle_read_notify_(CAClient &c, const CAHeader &h, const uint8_t *raw) {
  CAChannel *ch = this->find_channel_(c, h.p1);
  if (ch == nullptr) {
    this->send_error_(c, raw, 0, ECA_BADCHID, ERR_BAD_CHANNEL_ID);
    return;
  }
  if (!is_valid_read_type(h.dt)) {
    this->send_error_(c, raw, ch->cid, ECA_BADTYPE, ERR_BAD_DBR_TYPE);
    return;
  }
  const uint32_t count = h.count == 0 ? 1 : h.count;
  if (count > 1) {
    this->send_error_(c, raw, ch->cid, ECA_BADCOUNT, ERR_BAD_COUNT);
    return;
  }
  PVData d;
  ch->pv->fill_data_(ch->field, d);
  this->payload_buf_.clear();
  encode_dbr(this->payload_buf_, h.dt, d);
  this->queue_message_(c, CA_PROTO_READ_NOTIFY, h.dt, static_cast<uint16_t>(count), ECA_NORMAL, h.p2,
                       this->payload_buf_);
}

void EpicsCAServer::handle_write_(CAClient &c, const CAHeader &h, const uint8_t *raw, const uint8_t *payload) {
  const bool notify = h.cmd == CA_PROTO_WRITE_NOTIFY;
  const uint16_t reply_count = static_cast<uint16_t>(std::min<uint32_t>(h.count, CA_MAX_SHORT_COUNT));
  CAChannel *ch = this->find_channel_(c, h.p1);
  if (ch == nullptr) {
    this->send_error_(c, raw, 0, ECA_BADCHID, ERR_BAD_CHANNEL_ID);
    return;
  }
  auto finish = [&](uint32_t status, const char *msg) {
    if (notify) {
      this->queue_header_(c, CA_PROTO_WRITE_NOTIFY, h.dt, reply_count, status, h.p2);
    } else if (status != ECA_NORMAL) {
      this->send_error_(c, raw, ch->cid, status, msg);
    }
  };
  if (!this->can_write_(ch->pv, ch->field)) {
    finish(ECA_NOWTACCESS, "Write access denied");
    return;
  }
  if (h.dt == DBR_PUT_ACKT || h.dt == DBR_PUT_ACKS) {
    finish(ECA_NORMAL, "");  // no alarm acknowledgement model: accepted, no effect
    return;
  }
  if (h.dt > DBR_DOUBLE) {
    finish(ECA_BADTYPE, "Invalid DBR type for put");
    return;
  }
  if (h.count == 0) {
    finish(ECA_BADCOUNT, "Element count of zero");
    return;
  }
  double num;
  std::string str;
  bool is_string, integral;
  if (!decode_put(h.dt, h.count, payload, h.psize, ch->pv->type_ == NativeType::STRING, &num, &str, &is_string,
                  &integral)) {
    finish(ECA_PUTFAIL, "Put payload too short");
    return;
  }
  const uint32_t status = ch->pv->client_write_(num, str, is_string, integral);
  ESP_LOGD(TAG, "Client %s (%s@%s) wrote '%s': status %" PRIu32, c.peer, c.user.c_str(), c.host.c_str(),
           ch->pv->get_full_name().c_str(), status);
  finish(status, "Put failed");
}

void EpicsCAServer::handle_event_add_(CAClient &c, const CAHeader &h, const uint8_t *raw, const uint8_t *payload) {
  CAChannel *ch = this->find_channel_(c, h.p1);
  if (ch == nullptr) {
    this->send_error_(c, raw, 0, ECA_BADCHID, ERR_BAD_CHANNEL_ID);
    return;
  }
  if (!is_valid_read_type(h.dt)) {
    this->send_error_(c, raw, ch->cid, ECA_BADTYPE, ERR_BAD_DBR_TYPE);
    return;
  }
  const uint32_t count = h.count == 0 ? 1 : h.count;
  if (count > 1) {
    this->send_error_(c, raw, ch->cid, ECA_BADCOUNT, ERR_BAD_COUNT);
    return;
  }
  uint16_t mask = DBE_VALUE | DBE_ALARM;
  if (h.psize >= CA_EVENT_ADD_MIN_PAYLOAD)
    mask = rd16(payload + CA_EVENT_ADD_MASK_OFFSET);
  if (mask == 0)
    mask = DBE_VALUE | DBE_ALARM;

  CASubscription *sub = nullptr;
  for (auto &s : c.subs) {
    if (s.subid == h.p2) {
      sub = &s;
      break;
    }
  }
  if (sub == nullptr) {
    if (c.subs.size() >= this->max_subs_) {
      this->send_error_(c, raw, ch->cid, ECA_ALLOCMEM, "Too many subscriptions");
      return;
    }
    c.subs.push_back(CASubscription{});
    sub = &c.subs.back();
  }
  sub->subid = h.p2;
  sub->sid = ch->sid;
  sub->pv = ch->pv;
  sub->field = ch->field;
  sub->dbr_type = h.dt;
  sub->count = count;
  sub->mask = mask;
  sub->pending = false;
  // The initial update is always sent.
  if (c.events_off || !this->send_monitor_(c, *sub))
    sub->pending = true;
}

void EpicsCAServer::handle_event_cancel_(CAClient &c, const CAHeader &h, const uint8_t *raw) {
  auto it = std::find_if(c.subs.begin(), c.subs.end(), [&](const CASubscription &s) { return s.subid == h.p2; });
  if (it == c.subs.end()) {
    this->send_error_(c, raw, 0, ECA_BADMONID, "Invalid subscription ID");
    return;
  }
  // Confirmation: EVENT_ADD with empty payload, p1 = sid, p2 = subscription id.
  this->queue_header_(c, CA_PROTO_EVENT_ADD, h.dt, h.count, h.p1, h.p2);
  c.subs.erase(it);
}

void EpicsCAServer::send_error_(CAClient &c, const uint8_t *raw, uint32_t cid, uint32_t status, const char *msg) {
  std::vector<uint8_t> &payload = this->payload_buf_;
  payload.assign(raw, raw + CA_HEADER_SIZE);  // original request header
  payload.insert(payload.end(), msg, msg + strlen(msg));
  payload.push_back(0);
  this->queue_message_(c, CA_PROTO_ERROR, 0, 0, cid, status, payload);
  ESP_LOGD(TAG, "Client %s: error %" PRIu32 " (%s)", c.peer, status, msg);
}

bool EpicsCAServer::reserve_tx_(CAClient &c, size_t size) {
  if (c.closing)
    return false;
  const size_t pending = c.tx.size() - c.tx_off;
  if (pending + size > 2 * this->send_buffer_size_) {
    ESP_LOGW(TAG, "Client %s: send buffer overflow, disconnecting", c.peer);
    c.closing = true;
    return false;
  }
  return true;
}

void EpicsCAServer::queue_message_(CAClient &c, uint16_t cmd, uint16_t dt, uint16_t count, uint32_t p1, uint32_t p2,
                                   const std::vector<uint8_t> &payload) {
  if (this->reserve_tx_(c, CA_HEADER_SIZE + padded_payload_size(payload.size())))
    put_message(c.tx, cmd, dt, count, p1, p2, payload);
}

void EpicsCAServer::queue_header_(CAClient &c, uint16_t cmd, uint16_t dt, uint32_t count, uint32_t p1, uint32_t p2) {
  if (this->reserve_tx_(c, CA_HEADER_SIZE))
    put_header(c.tx, cmd, 0, dt, static_cast<uint16_t>(std::min<uint32_t>(count, CA_MAX_SHORT_COUNT)), p1, p2);
}

bool EpicsCAServer::send_monitor_(CAClient &c, CASubscription &s) {
  if (c.closing)
    return true;
  PVData d;
  s.pv->fill_data_(s.field, d);
  this->payload_buf_.clear();
  encode_dbr(this->payload_buf_, s.dbr_type, d);
  const size_t pending = c.tx.size() - c.tx_off;
  if (pending + CA_HEADER_SIZE + padded_payload_size(this->payload_buf_.size()) > this->send_buffer_size_)
    return false;  // keep it pending; the latest value is sent when there is room
  put_message(c.tx, CA_PROTO_EVENT_ADD, s.dbr_type, static_cast<uint16_t>(s.count), ECA_NORMAL, s.subid,
              this->payload_buf_);
  return true;
}

void EpicsCAServer::service_pending_(CAClient &c) {
  if (c.events_off)
    return;
  for (auto &s : c.subs) {
    if (!s.pending)
      continue;
    if (!this->send_monitor_(c, s))
      break;
    s.pending = false;
  }
}

void EpicsCAServer::post_event(EpicsPV *pv, PVField field, uint16_t mask) {
  for (auto &cp : this->clients_) {
    CAClient &c = *cp;
    if (c.closing)
      continue;
    for (auto &s : c.subs) {
      if (s.pv != pv || s.field != field || (s.mask & mask) == 0)
        continue;
      if (c.events_off) {
        s.pending = true;
        continue;
      }
      s.pending = !this->send_monitor_(c, s);
    }
  }
}

void EpicsCAServer::flush_(CAClient &c) {
  while (c.tx_off < c.tx.size()) {
    const ssize_t n = lwip_send(c.fd, c.tx.data() + c.tx_off, c.tx.size() - c.tx_off, 0);
    if (n > 0) {
      c.tx_off += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && would_block(errno))
      break;
    c.closing = true;
    return;
  }
  if (c.tx_off >= c.tx.size()) {
    c.tx.clear();
    c.tx_off = 0;
  } else if (c.tx_off >= TX_COMPACT_THRESHOLD) {
    c.tx.erase(c.tx.begin(), c.tx.begin() + c.tx_off);
    c.tx_off = 0;
  }
}

}  // namespace esphome::epics_ca

#endif  // USE_ESP32
