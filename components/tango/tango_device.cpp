#include "tango_device.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <limits>
#include <sys/time.h>

namespace esphome::tango {

// ======================================================================== //
//                               Constants                                  //
// ======================================================================== //

// CORBA::TCKind
static constexpr uint32_t TK_NULL = 0;
static constexpr uint32_t TK_VOID = 1;
static constexpr uint32_t TK_SHORT = 2;
static constexpr uint32_t TK_LONG = 3;
static constexpr uint32_t TK_USHORT = 4;
static constexpr uint32_t TK_ULONG = 5;
static constexpr uint32_t TK_FLOAT = 6;
static constexpr uint32_t TK_DOUBLE = 7;
static constexpr uint32_t TK_BOOLEAN = 8;
static constexpr uint32_t TK_OCTET = 10;
static constexpr uint32_t TK_ENUM = 17;
static constexpr uint32_t TK_STRING = 18;
static constexpr uint32_t TK_SEQUENCE = 19;
static constexpr uint32_t TK_ALIAS = 21;
static constexpr uint32_t TK_LONGLONG = 23;
static constexpr uint32_t TK_ULONGLONG = 24;

// Tango::AttributeDataType, the discriminant of Tango::AttrValUnion.
static constexpr uint32_t ATT_BOOL = 0;
static constexpr uint32_t ATT_SHORT = 1;
static constexpr uint32_t ATT_LONG = 2;
static constexpr uint32_t ATT_LONG64 = 3;
static constexpr uint32_t ATT_FLOAT = 4;
static constexpr uint32_t ATT_DOUBLE = 5;
static constexpr uint32_t ATT_UCHAR = 6;
static constexpr uint32_t ATT_USHORT = 7;
static constexpr uint32_t ATT_ULONG = 8;
static constexpr uint32_t ATT_ULONG64 = 9;
static constexpr uint32_t ATT_STRING = 10;
static constexpr uint32_t ATT_STATE = 11;
static constexpr uint32_t ATT_DEVICE_STATE = 12;
static constexpr uint32_t ATT_ENCODED = 13;
static constexpr uint32_t ATT_NO_DATA = 14;

// Tango::AttrDataFormat
static constexpr uint32_t FMT_SCALAR = 0;
static constexpr uint32_t FMT_UNKNOWN = 3;
// Tango::DispLevel
static constexpr uint32_t DL_OPERATOR = 0;
// Tango::DATA_TYPE_UNKNOWN
static constexpr int32_t DATA_TYPE_UNKNOWN = 100;
// The IDL version of the served devices (DevInfo::server_version).
static constexpr int32_t SERVED_IDL_VERSION = 6;

static constexpr const char *NOT_SPECIFIED = "Not specified";
static constexpr const char *NO_DESCRIPTION = "No description";
static constexpr const char *NO_STANDARD_UNIT = "No standard unit";
static constexpr const char *NO_DISPLAY_UNIT = "No display unit";
static constexpr const char *ASSOC_WRITE_NOT_SPEC = "None";
static constexpr const char *DESC_NOT_SET = "Uninitialised";
static constexpr const char *ALL_ATTRIBUTES = "All attributes";
static constexpr const char *ALL_ATTRIBUTES_3 = "All attributes_3";

// Tango::DevError reasons and origins.
static constexpr const char *REASON_ATTR_NOT_FOUND = "API_AttrNotFound";
static constexpr const char *REASON_INCOMPATIBLE_ATTR_TYPE = "API_IncompatibleAttrDataType";
static constexpr const char *REASON_ATTR_OUTSIDE_LIMIT = "API_WAttrOutsideLimit";
static constexpr const char *REASON_COMMAND_NOT_FOUND = "API_CommandNotFound";
static constexpr const char *REASON_INCOMPATIBLE_CMD_ARG = "API_IncompatibleCmdArgumentType";
static constexpr const char *ORIGIN_BLACK_BOX = "BlackBox::read";
static constexpr const char *ORIGIN_COMMAND_INOUT = "TangoDevice::command_inout";
static constexpr const char *ORIGIN_DISPATCH = "TangoDevice::dispatch";
static constexpr const char *ORIGIN_GET_ATTRIBUTE_CONFIG = "TangoDevice::get_attribute_config";
static constexpr const char *ORIGIN_READ_ATTRIBUTES = "TangoDevice::read_attributes";
static constexpr const char *ORIGIN_WRITE_ATTRIBUTE = "TangoDevice::write_attribute";
static constexpr const char *ORIGIN_WRITE_ATTRIBUTES = "TangoDevice::write_attributes";
static constexpr const char *ORIGIN_WRITE_READ_ATTRIBUTES = "TangoDevice::write_read_attributes";

// Operations of the IDL that this server has nothing to answer with.
static const char *const UNSUPPORTED_OPERATIONS[] = {
    "set_attribute_config",
    "set_attribute_config_3",
    "set_attribute_config_4",
    "set_attribute_config_5",
    "command_inout_history_2",
    "command_inout_history_4",
    "read_attribute_history_2",
    "read_attribute_history_3",
    "read_attribute_history_4",
    "read_attribute_history_5",
    "get_pipe_config_5",
    "set_pipe_config_5",
    "read_pipe_5",
    "write_pipe_5",
    "write_read_pipe_5",
};

// AttributeAlarm_3: min_alarm, max_alarm, min_warning, max_warning, delta_t, delta_val.
static constexpr int ALARM_PROPERTY_COUNT = 6;
// Size of the buffer used to format conversion error messages.
static constexpr size_t ERROR_TEXT_SIZE = 96;
// 2^64: magnitudes from here on do not fit an uint64_t.
static constexpr double TWO_POW_64 = 18446744073709551616.0;

// IOP::ComponentId of the code sets component, and the OSF code set registry ids it names.
static constexpr uint32_t IOP_TAG_CODE_SETS = 1;
static constexpr uint32_t CODESET_UTF8 = 0x05010001;
static constexpr uint32_t CODESET_ISO_8859_1 = 0x00010001;
static constexpr uint32_t CODESET_UTF16 = 0x00010109;

static constexpr const char *HEX_DIGITS = "0123456789ABCDEF";

static constexpr const char *REPO_DEV_FAILED = "IDL:Tango/DevFailed:1.0";
static constexpr const char *REPO_MULTI_DEV_FAILED = "IDL:Tango/MultiDevFailed:1.0";
static constexpr const char *REPO_DEV_STATE = "IDL:Tango/DevState:1.0";
static constexpr const char *REPO_CORBA_OBJECT = "IDL:omg.org/CORBA/Object:1.0";

// Every Tango IDL interface a served device implements, most derived first.
static const char *const REPO_DEVICES[] = {
    "IDL:Tango/Device_6:1.0", "IDL:Tango/Device_5:1.0", "IDL:Tango/Device_4:1.0",
    "IDL:Tango/Device_3:1.0", "IDL:Tango/Device_2:1.0", "IDL:Tango/Device:1.0",
};

static const char *const STATE_NAMES[DEV_STATE_COUNT] = {"ON",      "OFF",    "CLOSE",   "OPEN",   "INSERT",
                                                         "EXTRACT", "MOVING", "STANDBY", "FAULT",  "INIT",
                                                         "RUNNING", "ALARM",  "DISABLE", "UNKNOWN"};

const char *dev_state_name(DevState state) {
  auto i = static_cast<uint32_t>(state);
  return i < DEV_STATE_COUNT ? STATE_NAMES[i] : "UNKNOWN";
}

struct TypeInfo {
  const char *array_repo_id;
  const char *array_name;
  const char *tango_name;
  int32_t type;
  uint32_t tk;        // TypeCode kind of one element.
  uint32_t att;       // AttrValUnion discriminant.
  uint8_t elem_size;  // Smallest CDR size of one element (sequence length sanity check).
};

static const TypeInfo TYPES[] = {
    {"IDL:Tango/DevVarBooleanArray:1.0", "DevVarBooleanArray", "DevBoolean", DEV_BOOLEAN, TK_BOOLEAN, ATT_BOOL, 1},
    {"IDL:Tango/DevVarShortArray:1.0", "DevVarShortArray", "DevShort", DEV_SHORT, TK_SHORT, ATT_SHORT, 2},
    {"IDL:Tango/DevVarLongArray:1.0", "DevVarLongArray", "DevLong", DEV_LONG, TK_LONG, ATT_LONG, 4},
    {"IDL:Tango/DevVarFloatArray:1.0", "DevVarFloatArray", "DevFloat", DEV_FLOAT, TK_FLOAT, ATT_FLOAT, 4},
    {"IDL:Tango/DevVarDoubleArray:1.0", "DevVarDoubleArray", "DevDouble", DEV_DOUBLE, TK_DOUBLE, ATT_DOUBLE, 8},
    {"IDL:Tango/DevVarUShortArray:1.0", "DevVarUShortArray", "DevUShort", DEV_USHORT, TK_USHORT, ATT_USHORT, 2},
    {"IDL:Tango/DevVarULongArray:1.0", "DevVarULongArray", "DevULong", DEV_ULONG, TK_ULONG, ATT_ULONG, 4},
    {"IDL:Tango/DevVarStringArray:1.0", "DevVarStringArray", "DevString", DEV_STRING, TK_STRING, ATT_STRING, 4},
    {"IDL:Tango/DevVarStateArray:1.0", "DevVarStateArray", "DevState", DEV_STATE, TK_ENUM, ATT_STATE, 4},
    {"IDL:Tango/DevVarStringArray:1.0", "DevVarStringArray", "ConstDevString", CONST_DEV_STRING, TK_STRING, ATT_STRING,
     4},
    {"IDL:Tango/DevVarCharArray:1.0", "DevVarCharArray", "DevUChar", DEV_UCHAR, TK_OCTET, ATT_UCHAR, 1},
    {"IDL:Tango/DevVarLong64Array:1.0", "DevVarLong64Array", "DevLong64", DEV_LONG64, TK_LONGLONG, ATT_LONG64, 8},
    {"IDL:Tango/DevVarULong64Array:1.0", "DevVarULong64Array", "DevULong64", DEV_ULONG64, TK_ULONGLONG, ATT_ULONG64, 8},
    {"IDL:Tango/DevVarShortArray:1.0", "DevVarShortArray", "DevEnum", DEV_ENUM, TK_SHORT, ATT_SHORT, 2},
};

static const TypeInfo *type_info(int32_t type) {
  for (const auto &t : TYPES) {
    if (t.type == type)
      return &t;
  }
  return nullptr;
}

static int32_t type_from_att(uint32_t att) {
  switch (att) {
    case ATT_BOOL:
      return DEV_BOOLEAN;
    case ATT_SHORT:
      return DEV_SHORT;
    case ATT_LONG:
      return DEV_LONG;
    case ATT_LONG64:
      return DEV_LONG64;
    case ATT_FLOAT:
      return DEV_FLOAT;
    case ATT_DOUBLE:
      return DEV_DOUBLE;
    case ATT_UCHAR:
      return DEV_UCHAR;
    case ATT_USHORT:
      return DEV_USHORT;
    case ATT_ULONG:
      return DEV_ULONG;
    case ATT_ULONG64:
      return DEV_ULONG64;
    case ATT_STRING:
      return DEV_STRING;
    case ATT_STATE:
      return DEV_STATE;
    default:
      return DEV_VOID;
  }
}

static const char *type_name(int32_t type) {
  if (type == DEV_VOID)
    return "DevVoid";
  const TypeInfo *ti = type_info(type);
  return ti != nullptr ? ti->tango_name : "Unknown";
}

static bool iequals(const std::string &a, const std::string &b) {
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); i++) {
    if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i])))
      return false;
  }
  return true;
}

static std::string to_lower(const std::string &s) {
  std::string out(s);
  for (auto &c : out)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}

static void add_error(DevErrorList &err, const char *reason, const std::string &desc, const char *origin) {
  DevError e;
  e.reason = reason;
  e.severity = ERR_ERR;
  e.desc = desc;
  e.origin = origin;
  err.push_back(std::move(e));
}

static void add_attr_not_found(DevErrorList &err, const std::string &name, const char *origin) {
  add_error(err, REASON_ATTR_NOT_FOUND, "Attribute " + name + " not found", origin);
}

// ======================================================================== //
//                               TangoValue                                 //
// ======================================================================== //

TangoValue TangoValue::make_bool(bool v) {
  TangoValue r;
  r.type = DEV_BOOLEAN;
  r.b = v;
  return r;
}
TangoValue TangoValue::make_int(int32_t type, int64_t v) {
  TangoValue r;
  r.type = type;
  r.i = v;
  return r;
}
TangoValue TangoValue::make_uint(int32_t type, uint64_t v) {
  TangoValue r;
  r.type = type;
  r.u = v;
  return r;
}
TangoValue TangoValue::make_float(int32_t type, double v) {
  TangoValue r;
  r.type = type;
  r.d = v;
  return r;
}
TangoValue TangoValue::make_string(const std::string &v) {
  TangoValue r;
  r.type = DEV_STRING;
  r.s = v;
  return r;
}
TangoValue TangoValue::make_state(DevState v) {
  TangoValue r;
  r.type = DEV_STATE;
  r.u = static_cast<uint32_t>(v);
  return r;
}

static bool is_signed_type(int32_t t) { return t == DEV_SHORT || t == DEV_LONG || t == DEV_LONG64 || t == DEV_ENUM; }
static bool is_unsigned_type(int32_t t) {
  return t == DEV_UCHAR || t == DEV_USHORT || t == DEV_ULONG || t == DEV_ULONG64;
}
static bool is_float_type(int32_t t) { return t == DEV_FLOAT || t == DEV_DOUBLE; }

bool TangoValue::is_numeric() const {
  return is_signed_type(this->type) || is_unsigned_type(this->type) || is_float_type(this->type);
}

bool TangoValue::to_double(double &out) const {
  if (is_signed_type(this->type)) {
    out = static_cast<double>(this->i);
  } else if (is_unsigned_type(this->type)) {
    out = static_cast<double>(this->u);
  } else if (is_float_type(this->type)) {
    out = this->d;
  } else if (this->type == DEV_BOOLEAN) {
    out = this->b ? 1.0 : 0.0;
  } else {
    return false;
  }
  return true;
}

/// Convert `in` to the Tango type `target`, refusing anything lossy.
static bool convert_value(const TangoValue &in, int32_t target, TangoValue &out, std::string &why) {
  out = TangoValue();
  out.type = target == CONST_DEV_STRING ? DEV_STRING : target;
  char buf[ERROR_TEXT_SIZE];

  if (target == DEV_STRING || target == CONST_DEV_STRING) {
    if (in.type != DEV_STRING && in.type != CONST_DEV_STRING) {
      std::snprintf(buf, sizeof(buf), "Expected a string, received %s", type_name(in.type));
      why = buf;
      return false;
    }
    out.s = in.s;
    return true;
  }
  if (target == DEV_STATE) {
    if (in.type != DEV_STATE || in.u >= DEV_STATE_COUNT) {
      why = "Expected a DevState";
      return false;
    }
    out.u = in.u;
    return true;
  }
  if (target == DEV_BOOLEAN) {
    if (in.type == DEV_BOOLEAN) {
      out.b = in.b;
    } else if (is_signed_type(in.type)) {
      out.b = in.i != 0;
    } else if (is_unsigned_type(in.type)) {
      out.b = in.u != 0;
    } else {
      std::snprintf(buf, sizeof(buf), "Expected a DevBoolean, received %s", type_name(in.type));
      why = buf;
      return false;
    }
    return true;
  }
  if (is_float_type(target)) {
    double d;
    if (!in.to_double(d)) {
      std::snprintf(buf, sizeof(buf), "Expected a number, received %s", type_name(in.type));
      why = buf;
      return false;
    }
    if (target == DEV_FLOAT && std::isfinite(d) && std::fabs(d) > FLT_MAX) {
      why = "Value out of range for a DevFloat";
      return false;
    }
    out.d = target == DEV_FLOAT ? static_cast<double>(static_cast<float>(d)) : d;
    return true;
  }
  if (is_signed_type(target) || is_unsigned_type(target)) {
    // Bring the value to a (negative flag, magnitude) pair first.
    bool neg = false;
    uint64_t mag = 0;
    if (in.type == DEV_BOOLEAN) {
      mag = in.b ? 1 : 0;
    } else if (is_signed_type(in.type)) {
      neg = in.i < 0;
      mag = neg ? static_cast<uint64_t>(-(in.i + 1)) + 1 : static_cast<uint64_t>(in.i);
    } else if (is_unsigned_type(in.type)) {
      mag = in.u;
    } else if (is_float_type(in.type)) {
      if (!std::isfinite(in.d) || std::trunc(in.d) != in.d || std::fabs(in.d) >= TWO_POW_64) {
        why = "Expected an integer value";
        return false;
      }
      neg = in.d < 0;
      mag = static_cast<uint64_t>(std::fabs(in.d));
    } else {
      std::snprintf(buf, sizeof(buf), "Expected an integer, received %s", type_name(in.type));
      why = buf;
      return false;
    }
    if (is_signed_type(target)) {
      uint64_t pos_max;
      switch (target) {
        case DEV_LONG:
          pos_max = std::numeric_limits<int32_t>::max();
          break;
        case DEV_LONG64:
          pos_max = std::numeric_limits<int64_t>::max();
          break;
        default:  // DEV_SHORT, DEV_ENUM
          pos_max = std::numeric_limits<int16_t>::max();
          break;
      }
      const uint64_t neg_max = pos_max + 1;  // Two's complement: |min| = max + 1.
      if ((neg && mag > neg_max) || (!neg && mag > pos_max)) {
        std::snprintf(buf, sizeof(buf), "Value out of range for a %s", type_name(target));
        why = buf;
        return false;
      }
      out.i = neg ? -static_cast<int64_t>(mag - 1) - 1 : static_cast<int64_t>(mag);
    } else {
      uint64_t max;
      switch (target) {
        case DEV_UCHAR:
          max = std::numeric_limits<uint8_t>::max();
          break;
        case DEV_USHORT:
          max = std::numeric_limits<uint16_t>::max();
          break;
        case DEV_ULONG:
          max = std::numeric_limits<uint32_t>::max();
          break;
        default:
          max = std::numeric_limits<uint64_t>::max();
          break;
      }
      if ((neg && mag != 0) || mag > max) {
        std::snprintf(buf, sizeof(buf), "Value out of range for a %s", type_name(target));
        why = buf;
        return false;
      }
      out.u = mag;
    }
    return true;
  }
  why = "Unsupported data type";
  return false;
}

// ======================================================================== //
//                      Element / TypeCode marshalling                      //
// ======================================================================== //

static void write_elem(CdrWriter &w, int32_t type, const TangoValue &v) {
  switch (type) {
    case DEV_BOOLEAN:
      w.write_bool(v.b);
      break;
    case DEV_SHORT:
    case DEV_ENUM:
      w.write_short(static_cast<int16_t>(v.i));
      break;
    case DEV_LONG:
      w.write_long(static_cast<int32_t>(v.i));
      break;
    case DEV_LONG64:
      w.write_longlong(v.i);
      break;
    case DEV_FLOAT:
      w.write_float(static_cast<float>(v.d));
      break;
    case DEV_DOUBLE:
      w.write_double(v.d);
      break;
    case DEV_UCHAR:
      w.write_octet(static_cast<uint8_t>(v.u));
      break;
    case DEV_USHORT:
      w.write_ushort(static_cast<uint16_t>(v.u));
      break;
    case DEV_ULONG:
      w.write_ulong(static_cast<uint32_t>(v.u));
      break;
    case DEV_ULONG64:
      w.write_ulonglong(v.u);
      break;
    case DEV_STRING:
    case CONST_DEV_STRING:
      w.write_string(v.s);
      break;
    case DEV_STATE:
      w.write_ulong(static_cast<uint32_t>(v.u));
      break;
    default:
      break;
  }
}

static void read_elem(CdrReader &r, int32_t type, TangoValue &v) {
  v = TangoValue();
  v.type = type;
  switch (type) {
    case DEV_BOOLEAN:
      v.b = r.read_bool();
      break;
    case DEV_SHORT:
    case DEV_ENUM:
      v.i = r.read_short();
      break;
    case DEV_LONG:
      v.i = r.read_long();
      break;
    case DEV_LONG64:
      v.i = r.read_longlong();
      break;
    case DEV_FLOAT:
      v.d = r.read_float();
      break;
    case DEV_DOUBLE:
      v.d = r.read_double();
      break;
    case DEV_UCHAR:
      v.u = r.read_octet();
      break;
    case DEV_USHORT:
      v.u = r.read_ushort();
      break;
    case DEV_ULONG:
      v.u = r.read_ulong();
      break;
    case DEV_ULONG64:
      v.u = r.read_ulonglong();
      break;
    case DEV_STRING:
      r.read_string(v.s);
      break;
    case DEV_STATE:
      v.u = r.read_ulong();
      if (v.u >= DEV_STATE_COUNT)
        r.fail();
      break;
    default:
      r.fail();
      break;
  }
}

// TypeCode of Tango::DevState: tk_enum { repo id, name, member names }.
static void write_tc_dev_state(CdrWriter &w) {
  w.write_ulong(TK_ENUM);
  auto e = w.begin_encapsulation();
  w.write_string(REPO_DEV_STATE);
  w.write_string("DevState");
  w.write_ulong(DEV_STATE_COUNT);
  for (const char *name : STATE_NAMES)
    w.write_string(name);
  w.end_encapsulation(e);
}

// TypeCode of one scalar of a Tango type.
static void write_tc_scalar(CdrWriter &w, int32_t type) {
  const TypeInfo *ti = type_info(type);
  if (ti == nullptr) {
    w.write_ulong(TK_NULL);
    return;
  }
  if (ti->tk == TK_ENUM) {
    write_tc_dev_state(w);
  } else {
    w.write_ulong(ti->tk);
    if (ti->tk == TK_STRING)
      w.write_ulong(0);  // Unbounded.
  }
}

// TypeCode of a Tango::DevVar*Array: tk_alias { repo id, name, tk_sequence { element, bound 0 } }.
static void write_tc_array(CdrWriter &w, int32_t type) {
  const TypeInfo *ti = type_info(type);
  w.write_ulong(TK_ALIAS);
  auto alias = w.begin_encapsulation();
  w.write_string(ti->array_repo_id);
  w.write_string(ti->array_name);
  w.write_ulong(TK_SEQUENCE);
  auto seq = w.begin_encapsulation();
  write_tc_scalar(w, type);
  w.write_ulong(0);  // Unbounded.
  w.end_encapsulation(seq);
  w.end_encapsulation(alias);
}

/// What a received TypeCode describes, as far as Tango data goes.
struct TcDesc {
  bool is_void{false};
  bool is_seq{false};
  int32_t type{DEV_VOID};
};

/// Parse a TypeCode. Returns false for anything that is not a Tango scalar, a
/// sequence of Tango scalars, or void (the value can then not be skipped).
static bool parse_tc(CdrReader &r, TcDesc &d, int depth) {
  if (depth > 4)
    return false;
  uint32_t tk = r.read_ulong();
  if (!r.ok())
    return false;
  switch (tk) {
    case TK_NULL:
    case TK_VOID:
      d.is_void = true;
      return true;
    case TK_BOOLEAN:
      d.type = DEV_BOOLEAN;
      return true;
    case TK_SHORT:
      d.type = DEV_SHORT;
      return true;
    case TK_LONG:
      d.type = DEV_LONG;
      return true;
    case TK_USHORT:
      d.type = DEV_USHORT;
      return true;
    case TK_ULONG:
      d.type = DEV_ULONG;
      return true;
    case TK_FLOAT:
      d.type = DEV_FLOAT;
      return true;
    case TK_DOUBLE:
      d.type = DEV_DOUBLE;
      return true;
    case TK_OCTET:
      d.type = DEV_UCHAR;
      return true;
    case TK_LONGLONG:
      d.type = DEV_LONG64;
      return true;
    case TK_ULONGLONG:
      d.type = DEV_ULONG64;
      return true;
    case TK_STRING:
      r.read_ulong();  // Bound.
      d.type = DEV_STRING;
      return r.ok();
    case TK_ENUM: {
      CdrReader e;
      if (!r.read_encapsulation(e))
        return false;
      std::string repo_id;
      e.read_string(repo_id);
      if (!e.ok() || repo_id != REPO_DEV_STATE)
        return false;
      d.type = DEV_STATE;
      return true;
    }
    case TK_ALIAS: {
      CdrReader e;
      if (!r.read_encapsulation(e))
        return false;
      e.skip_string();  // Repository id.
      e.skip_string();  // Name.
      return parse_tc(e, d, depth + 1) && e.ok();
    }
    case TK_SEQUENCE: {
      if (d.is_seq)
        return false;  // No nested sequences in Tango data.
      CdrReader e;
      if (!r.read_encapsulation(e))
        return false;
      TcDesc elem;
      if (!parse_tc(e, elem, depth + 1) || elem.is_void || elem.is_seq)
        return false;
      e.read_ulong();  // Bound.
      if (!e.ok())
        return false;
      d.is_seq = true;
      d.type = elem.type;
      return true;
    }
    default:
      return false;  // tk_struct (DevVarLongStringArray, DevEncoded...), indirections, etc.
  }
}

/// Read the value of an Any whose TypeCode `d` was parsed.
static void read_any_value(CdrReader &r, const TcDesc &d, std::vector<TangoValue> &values) {
  values.clear();
  if (d.is_void)
    return;
  uint32_t n = 1;
  if (d.is_seq) {
    const TypeInfo *ti = type_info(d.type);
    n = r.read_seq_len(ti != nullptr ? ti->elem_size : 1);
  }
  for (uint32_t i = 0; i < n && r.ok(); i++) {
    TangoValue v;
    read_elem(r, d.type, v);
    values.push_back(std::move(v));
  }
}

static void write_time(CdrWriter &w) {
  // Tango::TimeVal { long tv_sec; long tv_usec; long tv_nsec; }
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  w.write_long(static_cast<int32_t>(tv.tv_sec));
  w.write_long(static_cast<int32_t>(tv.tv_usec));
  w.write_long(0);
}

static void write_errors(CdrWriter &w, const DevErrorList &err) {
  // Tango::DevErrorList = sequence<DevError { string reason; ErrSeverity severity; string desc; string origin; }>
  w.write_ulong(static_cast<uint32_t>(err.size()));
  for (const auto &e : err) {
    w.write_string(e.reason);
    w.write_ulong(e.severity);
    w.write_string(e.desc);
    w.write_string(e.origin);
  }
}

static void skip_errors(CdrReader &r) {
  uint32_t n = r.read_seq_len(16);
  for (uint32_t i = 0; i < n && r.ok(); i++) {
    r.skip_string();
    r.read_ulong();
    r.skip_string();
    r.skip_string();
  }
}

static void write_string_list(CdrWriter &w, const std::vector<std::string> &list) {
  w.write_ulong(static_cast<uint32_t>(list.size()));
  for (const auto &s : list)
    w.write_string(s);
}

/// Read one entry of an attribute value list sent to a write operation: the
/// Tango::AttributeValue of IDL 1 (`v4` false) or the Tango::AttributeValue_4.
/// `supported` is false when the data is of a kind this server cannot take.
static void read_input_attr_value(CdrReader &in, bool v4, std::string &name, std::vector<TangoValue> &values,
                                  bool &supported) {
  values.clear();
  supported = true;
  if (v4) {
    uint32_t disc = in.read_ulong();
    if (!in.ok())
      return;
    if (disc <= ATT_STATE) {
      int32_t type = type_from_att(disc);
      const TypeInfo *ti = type_info(type);
      uint32_t n = in.read_seq_len(ti->elem_size);
      for (uint32_t i = 0; i < n && in.ok(); i++) {
        TangoValue v;
        read_elem(in, type, v);
        values.push_back(std::move(v));
      }
    } else if (disc == ATT_DEVICE_STATE) {
      TangoValue v;
      read_elem(in, DEV_STATE, v);
      values.push_back(std::move(v));
    } else if (disc == ATT_ENCODED) {
      // DevVarEncodedArray = sequence<DevEncoded { DevString encoded_format; DevVarCharArray encoded_data; }>
      uint32_t n = in.read_seq_len(8);
      for (uint32_t i = 0; i < n && in.ok(); i++) {
        in.skip_string();
        in.skip(in.read_seq_len(1));
      }
      supported = false;
    } else if (disc == ATT_NO_DATA) {
      in.read_bool();
    } else {
      in.fail();
      return;
    }
  } else {
    TcDesc d;
    if (!parse_tc(in, d, 0)) {
      in.fail();  // The value cannot be skipped without understanding its TypeCode.
      return;
    }
    read_any_value(in, d, values);
  }
  in.read_ulong();  // quality
  if (v4)
    in.read_ulong();  // data_format
  in.read_long();     // time.tv_sec
  in.read_long();     // time.tv_usec
  in.read_long();     // time.tv_nsec
  in.read_string(name);
  if (v4) {
    in.read_long();  // r_dim.dim_x
    in.read_long();  // r_dim.dim_y
    in.read_long();  // w_dim.dim_x
    in.read_long();  // w_dim.dim_y
    skip_errors(in);
  } else {
    in.read_long();  // dim_x
    in.read_long();  // dim_y
  }
}

// ======================================================================== //
//                               TangoDevice                                //
// ======================================================================== //

TangoDevice::TangoDevice() {
  this->state_attr_.name = "State";
  this->state_attr_.data_type = DEV_STATE;
  this->state_attr_.writable = ATTR_READ;
  this->state_attr_.description = "The state of the device";
  this->state_attr_.label = "State";
  this->state_attr_.device_state = true;
  this->state_attr_.reader = [this](AttrReading &r) {
    r.valid = true;
    r.value = TangoValue::make_state(this->state_);
  };

  this->status_attr_.name = "Status";
  this->status_attr_.data_type = DEV_STRING;
  this->status_attr_.writable = ATTR_READ;
  this->status_attr_.description = "The status of the device";
  this->status_attr_.label = "Status";
  this->status_attr_.reader = [this](AttrReading &r) {
    r.valid = true;
    r.value = TangoValue::make_string(this->get_status());
  };

  TangoCommand init;
  init.name = "Init";
  init.handler = [this](const TangoValue &, TangoValue &, DevErrorList &) {
    if (this->on_init_)
      this->on_init_();
  };
  TangoCommand state;
  state.name = "State";
  state.out_type = DEV_STATE;
  state.out_desc = "Device state";
  state.handler = [this](const TangoValue &, TangoValue &out, DevErrorList &) {
    out = TangoValue::make_state(this->state_);
  };
  TangoCommand status;
  status.name = "Status";
  status.out_type = DEV_STRING;
  status.out_desc = "Device status";
  status.handler = [this](const TangoValue &, TangoValue &out, DevErrorList &) {
    out = TangoValue::make_string(this->get_status());
  };
  this->builtin_commands_.push_back(init);
  this->builtin_commands_.push_back(state);
  this->builtin_commands_.push_back(status);
}

void TangoDevice::set_name(const std::string &name) {
  this->name_ = name;
  this->key_ = to_lower(name);
}

std::string TangoDevice::get_status() const {
  if (!this->status_.empty())
    return this->status_;
  return std::string("The device is in ") + dev_state_name(this->state_) + " state.";
}

bool TangoDevice::add_attribute(const TangoAttribute &attr) {
  if (attr.name.empty() || this->lookup_attr_(attr.name) != nullptr || type_info(attr.data_type) == nullptr ||
      attr.data_type == CONST_DEV_STRING)
    return false;
  this->attrs_.push_back(attr);
  return true;
}

bool TangoDevice::add_command(const TangoCommand &cmd) {
  if (cmd.name.empty() || this->lookup_command_(cmd.name) != nullptr)
    return false;
  if (cmd.in_type != DEV_VOID && (type_info(cmd.in_type) == nullptr || cmd.in_type == CONST_DEV_STRING))
    return false;
  if (cmd.out_type != DEV_VOID && type_info(cmd.out_type) == nullptr)
    return false;
  this->commands_.push_back(cmd);
  return true;
}

TangoAttribute *TangoDevice::find_attribute(const std::string &name) { return this->lookup_attr_(name); }

TangoAttribute *TangoDevice::attr_at_(size_t index) {
  if (index < this->attrs_.size())
    return &this->attrs_[index];
  if (index == this->attrs_.size())
    return &this->state_attr_;
  if (index == this->attrs_.size() + 1)
    return &this->status_attr_;
  return nullptr;
}

TangoAttribute *TangoDevice::lookup_attr_(const std::string &name) {
  for (size_t i = 0; i < this->attr_count_(); i++) {
    TangoAttribute *a = this->attr_at_(i);
    if (iequals(a->name, name))
      return a;
  }
  return nullptr;
}

const TangoCommand *TangoDevice::lookup_command_(const std::string &name) const {
  for (const auto &c : this->builtin_commands_) {
    if (iequals(c.name, name))
      return &c;
  }
  for (const auto &c : this->commands_) {
    if (iequals(c.name, name))
      return &c;
  }
  return nullptr;
}

void TangoDevice::for_each_command_(const std::function<void(const TangoCommand &)> &fn) const {
  for (const auto &c : this->builtin_commands_)
    fn(c);
  for (const auto &c : this->commands_)
    fn(c);
}

bool TangoDevice::matches_key(const std::string &key) const { return iequals(key, this->key_); }

bool TangoDevice::read_names_(CdrReader &in, std::vector<std::string> &names, bool &all) {
  // Tango::DevVarStringArray
  names.clear();
  all = false;
  uint32_t n = in.read_seq_len(4);
  for (uint32_t i = 0; i < n && in.ok(); i++) {
    std::string s;
    in.read_string(s);
    if (s == ALL_ATTRIBUTES || s == ALL_ATTRIBUTES_3)
      all = true;
    names.push_back(std::move(s));
  }
  return in.ok();
}

void TangoDevice::read_attr_(TangoAttribute &attr, AttrReading &reading) {
  reading = AttrReading();
  if (attr.reader)
    attr.reader(reading);
  if (!reading.valid || reading.quality == ATTR_INVALID) {
    reading.valid = false;
    reading.quality = ATTR_INVALID;
    return;
  }
  TangoValue converted;
  std::string why;
  if (!convert_value(reading.value, attr.data_type, converted, why)) {
    reading.valid = false;
    reading.quality = ATTR_INVALID;
    return;
  }
  reading.value = std::move(converted);
}

bool TangoDevice::write_attr_(TangoAttribute &attr, const std::vector<TangoValue> &values, DevErrorList &err) {
  if ((attr.writable != ATTR_READ_WRITE && attr.writable != ATTR_WRITE) || !attr.writer) {
    add_error(err, "API_AttrNotWritable", "Attribute " + attr.name + " is not writable", ORIGIN_WRITE_ATTRIBUTE);
    return false;
  }
  if (values.size() != 1) {
    add_error(err, "API_AttrIncorrectDataNumber",
              "Attribute " + attr.name + " is a scalar: exactly one value must be written", ORIGIN_WRITE_ATTRIBUTE);
    return false;
  }
  TangoValue v;
  std::string why;
  if (!convert_value(values[0], attr.data_type, v, why)) {
    add_error(err, REASON_INCOMPATIBLE_ATTR_TYPE,
              "Incompatible attribute type for " + attr.name + " (" + type_name(attr.data_type) + "): " + why,
              ORIGIN_WRITE_ATTRIBUTE);
    return false;
  }
  if (attr.data_type == DEV_ENUM && (v.i < 0 || v.i >= static_cast<int64_t>(attr.enum_labels.size()))) {
    add_error(err, REASON_ATTR_OUTSIDE_LIMIT, "Enumeration value out of range for attribute " + attr.name,
              ORIGIN_WRITE_ATTRIBUTE);
    return false;
  }
  double d;
  if (v.to_double(d) && attr.data_type != DEV_BOOLEAN) {
    char *end;
    if (!attr.min_value.empty()) {
      double lo = std::strtod(attr.min_value.c_str(), &end);
      if (end != attr.min_value.c_str() && d < lo) {
        add_error(err, REASON_ATTR_OUTSIDE_LIMIT,
                  "Set value for attribute " + attr.name + " is below the minimum authorized (" + attr.min_value + ")",
                  ORIGIN_WRITE_ATTRIBUTE);
        return false;
      }
    }
    if (!attr.max_value.empty()) {
      double hi = std::strtod(attr.max_value.c_str(), &end);
      if (end != attr.max_value.c_str() && d > hi) {
        add_error(err, REASON_ATTR_OUTSIDE_LIMIT,
                  "Set value for attribute " + attr.name + " is above the maximum authorized (" + attr.max_value + ")",
                  ORIGIN_WRITE_ATTRIBUTE);
        return false;
      }
    }
  }
  attr.writer(v, err);
  if (!err.empty())
    return false;
  if (attr.track_set_point) {
    attr.set_point = std::move(v);
    attr.has_set_point = true;
  }
  return true;
}

void TangoDevice::write_attr_value_(CdrWriter &w, int version, TangoAttribute *attr, const std::string &name,
                                    const AttrReading *reading, const DevErrorList *err) {
  const bool has_data = attr != nullptr && reading != nullptr && reading->valid && (err == nullptr || err->empty());
  const int32_t type = attr != nullptr ? attr->data_type : DATA_TYPE_UNKNOWN;
  const bool device_state = attr != nullptr && attr->device_state;
  const bool with_w = has_data && !device_state && attr->writable == ATTR_READ_WRITE;
  const TangoValue *rv = has_data ? &reading->value : nullptr;
  const TangoValue *wv = with_w ? (attr->has_set_point ? &attr->set_point : rv) : nullptr;

  // ---- value: Tango::AttrValUnion (IDL 4+) or any (IDL 1..3) ---- //
  if (version >= 4) {
    if (!has_data) {
      w.write_ulong(ATT_NO_DATA);
      w.write_bool(true);
    } else if (device_state) {
      w.write_ulong(ATT_DEVICE_STATE);
      write_elem(w, DEV_STATE, *rv);
    } else {
      w.write_ulong(type_info(type)->att);
      w.write_ulong(with_w ? 2 : 1);
      write_elem(w, type, *rv);
      if (with_w)
        write_elem(w, type, *wv);
    }
  } else {
    if (!has_data) {
      w.write_ulong(TK_NULL);
    } else if (device_state) {
      write_tc_dev_state(w);
      write_elem(w, DEV_STATE, *rv);
    } else {
      write_tc_array(w, type);
      w.write_ulong(with_w ? 2 : 1);
      write_elem(w, type, *rv);
      if (with_w)
        write_elem(w, type, *wv);
    }
  }

  // ---- quality, data_format (4+), data_type (5+), time, name ---- //
  w.write_ulong(has_data ? reading->quality : ATTR_INVALID);
  if (version >= 4)
    w.write_ulong(attr != nullptr ? FMT_SCALAR : FMT_UNKNOWN);
  if (version >= 5)
    w.write_long(type);
  write_time(w);
  w.write_string(name);

  // ---- dimensions and errors ---- //
  if (version >= 3) {
    w.write_long(has_data ? 1 : 0);  // r_dim.dim_x
    w.write_long(0);                 // r_dim.dim_y
    w.write_long(with_w ? 1 : 0);    // w_dim.dim_x
    w.write_long(0);                 // w_dim.dim_y
    if (err != nullptr) {
      write_errors(w, *err);
    } else {
      w.write_ulong(0);
    }
  } else {
    w.write_long(has_data ? 1 : 0);  // dim_x
    w.write_long(0);                 // dim_y
  }
}

void TangoDevice::write_attr_config_(CdrWriter &w, int version, const TangoAttribute &attr) {
  const char *format = NOT_SPECIFIED;
  std::string fmt;
  if (!attr.format.empty()) {
    fmt = attr.format;
    format = fmt.c_str();
  } else if (is_float_type(attr.data_type)) {
    format = "%6.2f";
  } else if (is_signed_type(attr.data_type) || is_unsigned_type(attr.data_type)) {
    format = "%d";
  } else if (attr.data_type == DEV_STRING) {
    format = "%s";
  }
  const std::string &label = attr.label.empty() ? attr.name : attr.label;
  const char *description = attr.description.empty() ? NO_DESCRIPTION : attr.description.c_str();
  const char *min_value = attr.min_value.empty() ? NOT_SPECIFIED : attr.min_value.c_str();
  const char *max_value = attr.max_value.empty() ? NOT_SPECIFIED : attr.max_value.c_str();

  w.write_string(attr.name);
  w.write_ulong(attr.writable);
  w.write_ulong(FMT_SCALAR);
  w.write_long(attr.data_type);
  if (version >= 5) {
    w.write_bool(false);  // memorized
    w.write_bool(false);  // mem_init
  }
  w.write_long(1);  // max_dim_x
  w.write_long(0);  // max_dim_y
  w.write_string(description);
  w.write_string(label);
  w.write_string(attr.unit);
  w.write_string(NO_STANDARD_UNIT);
  w.write_string(NO_DISPLAY_UNIT);
  w.write_string(format);
  w.write_string(min_value);
  w.write_string(max_value);
  if (version <= 2) {
    w.write_string(NOT_SPECIFIED);  // min_alarm
    w.write_string(NOT_SPECIFIED);  // max_alarm
  }
  w.write_string(ASSOC_WRITE_NOT_SPEC);  // writable_attr_name
  if (version >= 2)
    w.write_ulong(DL_OPERATOR);  // level
  if (version >= 5) {
    w.write_string("");  // root_attr_name
    write_string_list(w, attr.enum_labels);
  }
  if (version >= 3) {
    // AttributeAlarm_3
    for (int i = 0; i < ALARM_PROPERTY_COUNT; i++)
      w.write_string(NOT_SPECIFIED);
    w.write_ulong(0);  // extensions
    // EventProperties_3: ChangeEventProp_3, PeriodicEventProp_3, ArchiveEventProp_3
    w.write_string(NOT_SPECIFIED);  // ch_event.rel_change
    w.write_string(NOT_SPECIFIED);  // ch_event.abs_change
    w.write_ulong(0);               // ch_event.extensions
    w.write_string(NOT_SPECIFIED);  // per_event.period
    w.write_ulong(0);               // per_event.extensions
    w.write_string(NOT_SPECIFIED);  // arch_event.rel_change
    w.write_string(NOT_SPECIFIED);  // arch_event.abs_change
    w.write_string(NOT_SPECIFIED);  // arch_event.period
    w.write_ulong(0);               // arch_event.extensions
  }
  w.write_ulong(0);  // extensions
  if (version >= 3)
    w.write_ulong(0);  // sys_extensions
}

void TangoDevice::write_cmd_info_(CdrWriter &w, int version, const TangoCommand &cmd) {
  // DevCmdInfo { string cmd_name; long cmd_tag; long in_type; long out_type; string in_type_desc; string out_type_desc;
  // } DevCmdInfo_2 adds DispLevel level right after cmd_name.
  w.write_string(cmd.name);
  if (version >= 2)
    w.write_ulong(DL_OPERATOR);
  w.write_long(0);
  w.write_long(cmd.in_type);
  w.write_long(cmd.out_type);
  w.write_string(cmd.in_desc.empty() ? DESC_NOT_SET : cmd.in_desc.c_str());
  w.write_string(cmd.out_desc.empty() ? DESC_NOT_SET : cmd.out_desc.c_str());
}

void TangoDevice::record_black_box_(const std::string &op, const std::string &detail, const std::string &peer) {
  char stamp[32] = "";
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t secs = tv.tv_sec;
  struct tm tm_buf;
  if (localtime_r(&secs, &tm_buf) != nullptr) {
    size_t n = std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm_buf);
    std::snprintf(stamp + n, sizeof(stamp) - n, ":%02d", static_cast<int>(tv.tv_usec / 10000));
  }
  BlackBoxEntry entry;
  const bool has_detail = !detail.empty();
  std::snprintf(entry.text, sizeof(entry.text), "%s : Operation %s%s%s%s requested from %s", stamp, op.c_str(),
                has_detail ? " (" : "", detail.c_str(), has_detail ? ")" : "", peer.empty() ? "unknown" : peer.c_str());
  if (this->black_box_.size() >= BLACK_BOX_DEPTH)
    this->black_box_.pop();  // Drop the oldest entry.
  this->black_box_.push(entry);
}

// ======================================================================== //
//                               Operations                                 //
// ======================================================================== //

TangoDevice::OpResult TangoDevice::op_info_(OpContext &ctx, int version) {
  // DevInfo { string dev_class; string server_id; string server_host; long server_version; string doc_url; }
  // DevInfo_3 adds string dev_type; DevInfo_6 adds KeyValuePairList version_info.
  CdrWriter &w = *ctx.out;
  w.write_string(this->class_name_);
  w.write_string(this->server_id_);
  w.write_string(this->server_host_);
  w.write_long(SERVED_IDL_VERSION);
  w.write_string(this->doc_url_);
  if (version >= 3)
    w.write_string(this->dev_type_);
  if (version >= 6) {
    w.write_ulong(static_cast<uint32_t>(this->version_info_.size()));
    for (const auto &kv : this->version_info_) {
      w.write_string(kv.first);
      w.write_string(kv.second);
    }
  }
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_black_box_(OpContext &ctx) {
  int32_t n = ctx.in.read_long();
  if (!ctx.in.ok())
    return OpResult::MARSHAL;
  if (n <= 0) {
    add_error(ctx.err, "API_BlackBoxArgument", "Argument to read black box out of range", ORIGIN_BLACK_BOX);
    return OpResult::DEV_FAILED;
  }
  if (this->black_box_.empty()) {
    add_error(ctx.err, "API_BlackBoxEmpty", "Nothing stored yet in black-box", ORIGIN_BLACK_BOX);
    return OpResult::DEV_FAILED;
  }
  // The ring buffer holds the oldest entry first; the black box is read newest first.
  const BlackBoxEntry *entries[BLACK_BOX_DEPTH];
  size_t stored = 0;
  for (const auto &entry : this->black_box_)
    entries[stored++] = &entry;
  size_t count = std::min(static_cast<size_t>(n), stored);
  ctx.out->write_ulong(static_cast<uint32_t>(count));
  for (size_t i = 0; i < count; i++)
    ctx.out->write_string(entries[stored - 1 - i]->text);
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_command_inout_(OpContext &ctx, int version) {
  // any command_inout(in string command, in any argin)
  // any command_inout_2(in string command, in any argin, in DevSource source)
  // any command_inout_4(in string command, in any argin, in DevSource source, in ClntIdent cl_ident)
  // Nothing after the argument is needed: DevSource has no meaning without polling,
  // and the ClntIdent is not used by this server.
  (void) version;
  std::string name;
  ctx.in.read_string(name);
  if (!ctx.in.ok())
    return OpResult::MARSHAL;
  TcDesc d;
  bool understood = parse_tc(ctx.in, d, 0);
  std::vector<TangoValue> values;
  if (understood)
    read_any_value(ctx.in, d, values);
  if (understood && !ctx.in.ok())
    return OpResult::MARSHAL;
  ctx.bb_detail = "cmd = " + name;

  const TangoCommand *cmd = this->lookup_command_(name);
  if (cmd == nullptr) {
    add_error(ctx.err, REASON_COMMAND_NOT_FOUND, "Command " + name + " not found", ORIGIN_COMMAND_INOUT);
    return OpResult::DEV_FAILED;
  }

  TangoValue in;
  if (cmd->in_type != DEV_VOID) {
    int32_t expected = cmd->in_type == DEV_ENUM ? DEV_SHORT : cmd->in_type;
    if (!understood || d.is_void || d.is_seq || d.type != expected || values.size() != 1) {
      add_error(ctx.err, REASON_INCOMPATIBLE_CMD_ARG,
                std::string("Incompatible command argument type, expected type is : Tango::") + type_name(cmd->in_type),
                ORIGIN_COMMAND_INOUT);
      return OpResult::DEV_FAILED;
    }
    in = values[0];
    if (cmd->in_type == DEV_ENUM)
      in.type = DEV_ENUM;
  }

  TangoValue out;
  out.type = cmd->out_type;
  if (cmd->handler)
    cmd->handler(in, out, ctx.err);
  if (!ctx.err.empty())
    return OpResult::DEV_FAILED;

  CdrWriter &w = *ctx.out;
  if (cmd->out_type == DEV_VOID) {
    w.write_ulong(TK_NULL);
    return OpResult::OK;
  }
  TangoValue converted;
  std::string why;
  if (!convert_value(out, cmd->out_type, converted, why)) {
    add_error(ctx.err, REASON_INCOMPATIBLE_CMD_ARG,
              "Command " + cmd->name + " produced a value of the wrong type: " + why, ORIGIN_COMMAND_INOUT);
    return OpResult::DEV_FAILED;
  }
  write_tc_scalar(w, cmd->out_type);
  write_elem(w, cmd->out_type, converted);
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_command_query_(OpContext &ctx, int version) {
  std::string name;
  ctx.in.read_string(name);
  if (!ctx.in.ok())
    return OpResult::MARSHAL;
  const TangoCommand *cmd = this->lookup_command_(name);
  if (cmd == nullptr) {
    add_error(ctx.err, REASON_COMMAND_NOT_FOUND, "Command " + name + " not found", "TangoDevice::command_query");
    return OpResult::DEV_FAILED;
  }
  this->write_cmd_info_(*ctx.out, version, *cmd);
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_command_list_query_(OpContext &ctx, int version) {
  size_t count = this->builtin_commands_.size() + this->commands_.size();
  ctx.out->write_ulong(static_cast<uint32_t>(count));
  this->for_each_command_([&](const TangoCommand &c) { this->write_cmd_info_(*ctx.out, version, c); });
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_read_attributes_(OpContext &ctx, int version) {
  // AttributeValueList   read_attributes  (in DevVarStringArray names)
  // AttributeValueList   read_attributes_2(in DevVarStringArray names, in DevSource source)
  // AttributeValueList_3 read_attributes_3(in DevVarStringArray names, in DevSource source)
  // AttributeValueList_4 read_attributes_4(in DevVarStringArray names, in DevSource source, in ClntIdent cl_ident)
  // AttributeValueList_5 read_attributes_5(in DevVarStringArray names, in DevSource source, in ClntIdent cl_ident)
  std::vector<std::string> names;
  bool all;
  if (!this->read_names_(ctx.in, names, all))
    return OpResult::MARSHAL;
  if (version >= 2) {
    ctx.in.read_ulong();  // DevSource: always answered from the device.
    if (!ctx.in.ok())
      return OpResult::MARSHAL;
  }

  std::vector<std::pair<TangoAttribute *, std::string>> targets;
  if (all) {
    for (size_t i = 0; i < this->attr_count_(); i++)
      targets.emplace_back(this->attr_at_(i), this->attr_at_(i)->name);
  } else {
    for (const auto &n : names)
      targets.emplace_back(this->lookup_attr_(n), n);
  }

  // IDL 1 and 2 have no per-attribute error: an unknown attribute fails the whole call.
  if (version < 3) {
    for (const auto &t : targets) {
      if (t.first == nullptr) {
        add_attr_not_found(ctx.err, t.second, ORIGIN_READ_ATTRIBUTES);
        return OpResult::DEV_FAILED;
      }
    }
  }

  CdrWriter &w = *ctx.out;
  w.write_ulong(static_cast<uint32_t>(targets.size()));
  for (auto &t : targets) {
    if (t.first == nullptr) {
      DevErrorList err;
      add_attr_not_found(err, t.second, ORIGIN_READ_ATTRIBUTES);
      this->write_attr_value_(w, version, nullptr, t.second, nullptr, &err);
      continue;
    }
    AttrReading reading;
    this->read_attr_(*t.first, reading);
    this->write_attr_value_(w, version, t.first, t.second, &reading, nullptr);
  }
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_write_attributes_(OpContext &ctx, int version) {
  // void write_attributes  (in AttributeValueList values)
  // void write_attributes_3(in AttributeValueList values)
  // void write_attributes_4(in AttributeValueList_4 values, in ClntIdent cl_ident)
  const bool v4 = version >= 4;
  uint32_t n = ctx.in.read_seq_len(24);
  struct Entry {
    std::string name;
    std::vector<TangoValue> values;
    bool supported;
  };
  std::vector<Entry> entries;
  for (uint32_t i = 0; i < n && ctx.in.ok(); i++) {
    Entry e;
    read_input_attr_value(ctx.in, v4, e.name, e.values, e.supported);
    entries.push_back(std::move(e));
  }
  // Everything is parsed before anything is written.
  if (!ctx.in.ok())
    return OpResult::MARSHAL;

  for (size_t i = 0; i < entries.size(); i++) {
    Entry &e = entries[i];
    DevErrorList err;
    TangoAttribute *attr = this->lookup_attr_(e.name);
    if (attr == nullptr) {
      add_attr_not_found(err, e.name, ORIGIN_WRITE_ATTRIBUTES);
    } else if (!e.supported) {
      add_error(err, REASON_INCOMPATIBLE_ATTR_TYPE, "Unsupported data written to attribute " + e.name,
                ORIGIN_WRITE_ATTRIBUTES);
    } else {
      this->write_attr_(*attr, e.values, err);
    }
    if (!err.empty()) {
      NamedDevError ne;
      ne.name = e.name;
      ne.index_in_call = static_cast<int32_t>(i);
      ne.err_list = std::move(err);
      ctx.multi.push_back(std::move(ne));
    }
  }
  if (!ctx.multi.empty()) {
    if (!v4) {
      // write_attributes / write_attributes_3 answer with the first failure.
      ctx.err = ctx.multi.front().err_list;
      ctx.multi.clear();
      return OpResult::DEV_FAILED;
    }
    return OpResult::MULTI_DEV_FAILED;
  }
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_write_read_attributes_(OpContext &ctx, int version) {
  // AttributeValueList_4 write_read_attributes_4(in AttributeValueList_4 values, in ClntIdent cl_ident)
  // AttributeValueList_5 write_read_attributes_5(in AttributeValueList_4 values, in DevVarStringArray r_names,
  //                                              in ClntIdent cl_ident)
  uint32_t n = ctx.in.read_seq_len(24);
  std::vector<std::pair<std::string, std::vector<TangoValue>>> entries;
  std::vector<bool> supported;
  for (uint32_t i = 0; i < n && ctx.in.ok(); i++) {
    std::string name;
    std::vector<TangoValue> values;
    bool sup;
    read_input_attr_value(ctx.in, true, name, values, sup);
    entries.emplace_back(std::move(name), std::move(values));
    supported.push_back(sup);
  }
  std::vector<std::string> r_names;
  bool all = false;
  if (version >= 5 && ctx.in.ok())
    this->read_names_(ctx.in, r_names, all);
  if (!ctx.in.ok())
    return OpResult::MARSHAL;

  for (size_t i = 0; i < entries.size(); i++) {
    DevErrorList err;
    TangoAttribute *attr = this->lookup_attr_(entries[i].first);
    if (attr == nullptr) {
      add_attr_not_found(err, entries[i].first, ORIGIN_WRITE_READ_ATTRIBUTES);
    } else if (!supported[i]) {
      add_error(err, REASON_INCOMPATIBLE_ATTR_TYPE, "Unsupported data written to attribute " + entries[i].first,
                ORIGIN_WRITE_READ_ATTRIBUTES);
    } else {
      this->write_attr_(*attr, entries[i].second, err);
    }
    if (!err.empty()) {
      NamedDevError ne;
      ne.name = entries[i].first;
      ne.index_in_call = static_cast<int32_t>(i);
      ne.err_list = std::move(err);
      ctx.multi.push_back(std::move(ne));
    }
  }
  if (!ctx.multi.empty())
    return OpResult::MULTI_DEV_FAILED;

  // Read back: the r_names given to v5, or else the attributes that were written.
  std::vector<std::pair<TangoAttribute *, std::string>> targets;
  if (all) {
    for (size_t i = 0; i < this->attr_count_(); i++)
      targets.emplace_back(this->attr_at_(i), this->attr_at_(i)->name);
  } else if (!r_names.empty()) {
    for (const auto &rn : r_names)
      targets.emplace_back(this->lookup_attr_(rn), rn);
  } else {
    for (const auto &e : entries)
      targets.emplace_back(this->lookup_attr_(e.first), e.first);
  }
  CdrWriter &w = *ctx.out;
  w.write_ulong(static_cast<uint32_t>(targets.size()));
  for (auto &t : targets) {
    if (t.first == nullptr) {
      DevErrorList err;
      add_attr_not_found(err, t.second, ORIGIN_WRITE_READ_ATTRIBUTES);
      this->write_attr_value_(w, version, nullptr, t.second, nullptr, &err);
      continue;
    }
    AttrReading reading;
    this->read_attr_(*t.first, reading);
    this->write_attr_value_(w, version, t.first, t.second, &reading, nullptr);
  }
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::op_get_attribute_config_(OpContext &ctx, int version) {
  // AttributeConfigList   get_attribute_config  (in DevVarStringArray names)
  // AttributeConfigList_2 get_attribute_config_2(in DevVarStringArray names)
  // AttributeConfigList_3 get_attribute_config_3(in DevVarStringArray names)
  // AttributeConfigList_5 get_attribute_config_5(in DevVarStringArray names)
  std::vector<std::string> names;
  bool all;
  if (!this->read_names_(ctx.in, names, all))
    return OpResult::MARSHAL;
  std::vector<TangoAttribute *> targets;
  if (all) {
    for (size_t i = 0; i < this->attr_count_(); i++)
      targets.push_back(this->attr_at_(i));
  } else {
    for (const auto &n : names) {
      TangoAttribute *a = this->lookup_attr_(n);
      if (a == nullptr) {
        add_attr_not_found(ctx.err, n, ORIGIN_GET_ATTRIBUTE_CONFIG);
        return OpResult::DEV_FAILED;
      }
      targets.push_back(a);
    }
  }
  ctx.out->write_ulong(static_cast<uint32_t>(targets.size()));
  for (auto *a : targets)
    this->write_attr_config_(*ctx.out, version, *a);
  return OpResult::OK;
}

TangoDevice::OpResult TangoDevice::dispatch_(const std::string &op, OpContext &ctx) {
  CdrWriter &w = *ctx.out;

  // CORBA attributes of the Device interface.
  if (op == "_get_name") {
    w.write_string(this->name_);
    return OpResult::OK;
  }
  if (op == "_get_description") {
    w.write_string(this->description_);
    return OpResult::OK;
  }
  if (op == "_get_adm_name") {
    w.write_string("dserver/" + this->server_id_);
    return OpResult::OK;
  }
  if (op == "_get_state") {
    w.write_ulong(static_cast<uint32_t>(this->state_));
    return OpResult::OK;
  }
  if (op == "_get_status") {
    w.write_string(this->get_status());
    return OpResult::OK;
  }
  if (op == "ping")
    return OpResult::OK;

  if (op == "info")
    return this->op_info_(ctx, 1);
  if (op == "info_3")
    return this->op_info_(ctx, 3);
  if (op == "info_6")
    return this->op_info_(ctx, 6);
  if (op == "black_box")
    return this->op_black_box_(ctx);

  if (op == "command_inout")
    return this->op_command_inout_(ctx, 1);
  if (op == "command_inout_2")
    return this->op_command_inout_(ctx, 2);
  if (op == "command_inout_4")
    return this->op_command_inout_(ctx, 4);
  if (op == "command_query")
    return this->op_command_query_(ctx, 1);
  if (op == "command_query_2")
    return this->op_command_query_(ctx, 2);
  if (op == "command_list_query")
    return this->op_command_list_query_(ctx, 1);
  if (op == "command_list_query_2")
    return this->op_command_list_query_(ctx, 2);

  if (op == "read_attributes")
    return this->op_read_attributes_(ctx, 1);
  if (op == "read_attributes_2")
    return this->op_read_attributes_(ctx, 2);
  if (op == "read_attributes_3")
    return this->op_read_attributes_(ctx, 3);
  if (op == "read_attributes_4")
    return this->op_read_attributes_(ctx, 4);
  if (op == "read_attributes_5")
    return this->op_read_attributes_(ctx, 5);

  // write_attributes_3 takes the very same AttributeValueList as write_attributes.
  if (op == "write_attributes" || op == "write_attributes_3")
    return this->op_write_attributes_(ctx, 1);
  if (op == "write_attributes_4")
    return this->op_write_attributes_(ctx, 4);
  if (op == "write_read_attributes_4")
    return this->op_write_read_attributes_(ctx, 4);
  if (op == "write_read_attributes_5")
    return this->op_write_read_attributes_(ctx, 5);

  if (op == "get_attribute_config")
    return this->op_get_attribute_config_(ctx, 1);
  if (op == "get_attribute_config_2")
    return this->op_get_attribute_config_(ctx, 2);
  if (op == "get_attribute_config_3")  // Also what IDL 4 clients use.
    return this->op_get_attribute_config_(ctx, 3);
  if (op == "get_attribute_config_5")
    return this->op_get_attribute_config_(ctx, 5);

  for (const char *u : UNSUPPORTED_OPERATIONS) {
    if (op == u) {
      add_error(ctx.err, "API_NotSupportedFeature", "Operation " + op + " is not supported by this device server",
                ORIGIN_DISPATCH);
      return OpResult::DEV_FAILED;
    }
  }
  return OpResult::BAD_OPERATION;
}

void TangoDevice::handle_request(const GiopRequest &req, const std::string &peer, std::vector<uint8_t> &tx) {
  GiopReplyBuilder &rb = this->reply_;
  const uint8_t minor = req.minor;
  const uint32_t id = req.request_id;

  if (!req.key_resolved) {
    if (minor >= 2) {
      // Ask the client to address the object by its key.
      rb.begin(minor, id, REPLY_NEEDS_ADDRESSING_MODE);
      rb.writer().write_short(ADDR_KEY);
      rb.finish();
    } else {
      rb.system_exception(minor, id, sysex::OBJECT_NOT_EXIST, 0, COMPLETED_NO);
    }
  } else if (!this->matches_key(req.object_key)) {
    rb.system_exception(minor, id, sysex::OBJECT_NOT_EXIST, 0, COMPLETED_NO);
  } else if (req.operation == "_is_a") {
    CdrReader in = req.body();
    std::string repo_id;
    in.read_string(repo_id);
    if (!in.ok()) {
      rb.system_exception(minor, id, sysex::MARSHAL, 0, COMPLETED_NO);
    } else {
      bool is_a = repo_id == REPO_CORBA_OBJECT;
      for (const char *r : REPO_DEVICES)
        is_a = is_a || repo_id == r;
      rb.begin(minor, id, REPLY_NO_EXCEPTION);
      rb.writer().write_bool(is_a);
      rb.finish();
    }
  } else if (req.operation == "_non_existent" || req.operation == "_not_existent") {
    rb.begin(minor, id, REPLY_NO_EXCEPTION);
    rb.writer().write_bool(false);
    rb.finish();
  } else if (req.operation == "_interface" || req.operation == "_get_interface") {
    rb.system_exception(minor, id, sysex::NO_IMPLEMENT, 0, COMPLETED_NO);
  } else {
    OpContext ctx;
    ctx.in = req.body();
    rb.begin(minor, id, REPLY_NO_EXCEPTION);
    ctx.out = &rb.writer();
    OpResult res = this->dispatch_(req.operation, ctx);
    switch (res) {
      case OpResult::OK:
        rb.finish();
        break;
      case OpResult::DEV_FAILED:
        if (ctx.err.empty())
          add_error(ctx.err, "API_InternalError", "Unknown failure", ORIGIN_DISPATCH);
        rb.begin(minor, id, REPLY_USER_EXCEPTION);
        rb.writer().write_string(REPO_DEV_FAILED);
        write_errors(rb.writer(), ctx.err);
        rb.finish();
        break;
      case OpResult::MULTI_DEV_FAILED:
        // MultiDevFailed { NamedDevErrorList errors; }
        // NamedDevError { string name; long index_in_call; DevErrorList err_list; }
        rb.begin(minor, id, REPLY_USER_EXCEPTION);
        rb.writer().write_string(REPO_MULTI_DEV_FAILED);
        rb.writer().write_ulong(static_cast<uint32_t>(ctx.multi.size()));
        for (const auto &ne : ctx.multi) {
          rb.writer().write_string(ne.name);
          rb.writer().write_long(ne.index_in_call);
          write_errors(rb.writer(), ne.err_list);
        }
        rb.finish();
        break;
      case OpResult::BAD_OPERATION:
        rb.system_exception(minor, id, sysex::BAD_OPERATION, 0, COMPLETED_NO);
        break;
      case OpResult::MARSHAL:
        rb.system_exception(minor, id, sysex::MARSHAL, 0, COMPLETED_NO);
        break;
      case OpResult::NO_IMPLEMENT:
        rb.system_exception(minor, id, sysex::NO_IMPLEMENT, 0, COMPLETED_NO);
        break;
    }
    if (res != OpResult::BAD_OPERATION && res != OpResult::MARSHAL && req.operation != "black_box")
      this->record_black_box_(req.operation, ctx.bb_detail, peer);
  }

  if (req.response_expected)
    tx.insert(tx.end(), rb.bytes().begin(), rb.bytes().end());
}

void TangoDevice::handle_locate(const GiopRequest &req, std::vector<uint8_t> &tx) {
  GiopReplyBuilder &rb = this->reply_;
  if (!req.key_resolved && req.minor >= 2) {
    rb.begin_locate(req.minor, req.request_id, LOCATE_LOC_NEEDS_ADDRESSING_MODE);
    rb.writer().write_short(ADDR_KEY);
  } else {
    bool here = req.key_resolved && this->matches_key(req.object_key);
    rb.begin_locate(req.minor, req.request_id, here ? LOCATE_OBJECT_HERE : LOCATE_UNKNOWN_OBJECT);
  }
  rb.finish();
  tx.insert(tx.end(), rb.bytes().begin(), rb.bytes().end());
}

std::string TangoDevice::build_ior(const std::string &host, uint16_t port) const {
  // The stringified IOR is the hex form of an encapsulated IOP::IOR:
  // IOR { string type_id; sequence<TaggedProfile { ulong tag; sequence<octet> profile_data; }> profiles; }
  CdrWriter w;
  w.write_bool(HOST_LITTLE_ENDIAN);  // Byte order of the encapsulation (origin is this octet).
  w.write_string(REPO_DEVICES[0]);
  w.write_ulong(1);  // One profile.
  w.write_ulong(IOP_TAG_INTERNET_IOP);
  auto profile = w.begin_encapsulation();
  // IIOP::ProfileBody_1_1 { Version iiop_version; string host; ushort port; sequence<octet> object_key;
  //                         sequence<TaggedComponent> components; }
  w.write_octet(1);
  w.write_octet(2);
  w.write_string(host);
  w.write_ushort(port);
  w.write_octet_seq(this->key_.data(), this->key_.size());
  w.write_ulong(1);  // One component.
  w.write_ulong(IOP_TAG_CODE_SETS);
  auto codesets = w.begin_encapsulation();
  // CodeSetComponentInfo { CodeSetComponent ForCharData; CodeSetComponent ForWcharData; }
  // CodeSetComponent { ulong native_code_set; sequence<ulong> conversion_code_sets; }
  w.write_ulong(CODESET_UTF8);
  w.write_ulong(1);
  w.write_ulong(CODESET_ISO_8859_1);
  w.write_ulong(CODESET_UTF16);
  w.write_ulong(0);
  w.end_encapsulation(codesets);
  w.end_encapsulation(profile);

  std::string out = "IOR:";
  out.reserve(4 + w.buffer().size() * 2);
  for (uint8_t b : w.buffer()) {
    out.push_back(HEX_DIGITS[b >> 4]);
    out.push_back(HEX_DIGITS[b & 0x0F]);
  }
  return out;
}

// ======================================================================== //
//                              GiopConnection                              //
// ======================================================================== //

bool GiopConnection::feed(const uint8_t *data, size_t len, std::vector<uint8_t> &tx) {
  this->rx_.insert(this->rx_.end(), data, data + len);
  size_t off = 0;
  bool keep = true;
  while (keep && this->rx_.size() - off >= GIOP_HEADER_LEN) {
    const uint8_t *p = this->rx_.data() + off;
    GiopHeader h;
    if (!parse_giop_header(p, h)) {
      giop_write_header_only(tx, 2, GIOP_MESSAGE_ERROR);
      keep = false;
      break;
    }
    if (h.size > this->max_message_ - GIOP_HEADER_LEN) {
      // Too large for this device: refuse and drop the connection, the stream
      // cannot be resynchronised without reading the whole message.
      giop_write_header_only(tx, h.major == 1 && h.minor <= 2 ? h.minor : 2, GIOP_MESSAGE_ERROR);
      keep = false;
      break;
    }
    size_t total = GIOP_HEADER_LEN + h.size;
    if (this->rx_.size() - off < total)
      break;
    if (h.major != 1 || h.minor > 2) {
      giop_write_header_only(tx, 2, GIOP_MESSAGE_ERROR);  // Version not spoken here.
    } else {
      this->last_minor_ = h.minor;
      keep = this->process_(h, p, total, tx);
    }
    off += total;
  }
  if (!keep) {
    this->rx_.clear();
  } else if (off > 0) {
    this->rx_.erase(this->rx_.begin(), this->rx_.begin() + off);
  }
  return keep;
}

static uint32_t read_u32_at(const uint8_t *p, bool little) {
  if (little)
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

bool GiopConnection::process_(const GiopHeader &h, const uint8_t *msg, size_t len, std::vector<uint8_t> &tx) {
  switch (h.type) {
    case GIOP_REQUEST: {
      if (h.fragment_follows()) {
        if (this->frag_active_) {
          giop_write_header_only(tx, h.minor, GIOP_MESSAGE_ERROR);
          return false;
        }
        this->frag_.assign(msg, msg + len);
        this->frag_active_ = true;
        this->frag_minor_ = h.minor;
        // GIOP 1.2: the request id is the first member of the request header.
        this->frag_request_id_ = (h.minor >= 2 && len >= GIOP_REQUEST_ID_OFFSET + 4)
                                     ? read_u32_at(msg + GIOP_REQUEST_ID_OFFSET, h.little_endian())
                                     : 0;
        return true;
      }
      return this->process_request_(h, msg, len, tx);
    }
    case GIOP_FRAGMENT: {
      if (h.minor < 1 || !this->frag_active_ || h.minor != this->frag_minor_) {
        giop_write_header_only(tx, h.minor, GIOP_MESSAGE_ERROR);
        return false;
      }
      size_t data_off = GIOP_HEADER_LEN;
      if (h.minor >= 2) {
        // FragmentHeader_1_2 { ulong request_id; }
        if (len < GIOP_REQUEST_ID_OFFSET + 4 ||
            read_u32_at(msg + GIOP_REQUEST_ID_OFFSET, h.little_endian()) != this->frag_request_id_) {
          giop_write_header_only(tx, h.minor, GIOP_MESSAGE_ERROR);
          return false;
        }
        data_off = GIOP_REQUEST_ID_OFFSET + 4;
      }
      if (this->frag_.size() + (len - data_off) > this->max_message_) {
        giop_write_header_only(tx, h.minor, GIOP_MESSAGE_ERROR);
        return false;
      }
      this->frag_.insert(this->frag_.end(), msg + data_off, msg + len);
      if (h.fragment_follows())
        return true;
      this->frag_active_ = false;
      std::vector<uint8_t> whole;
      whole.swap(this->frag_);
      GiopHeader wh;
      parse_giop_header(whole.data(), wh);
      wh.flags &= static_cast<uint8_t>(~GIOP_FLAG_FRAGMENT);
      wh.size = static_cast<uint32_t>(whole.size() - GIOP_HEADER_LEN);
      return this->process_request_(wh, whole.data(), whole.size(), tx);
    }
    case GIOP_LOCATE_REQUEST: {
      GiopRequest req;
      if (!parse_giop_locate_request(msg, len, h, req)) {
        giop_write_header_only(tx, h.minor, GIOP_MESSAGE_ERROR);
        return false;
      }
      this->device_->handle_locate(req, tx);
      return true;
    }
    case GIOP_CANCEL_REQUEST: {
      // Requests are answered synchronously, so there is never anything to cancel,
      // except a fragmented request still being received.
      if (len >= GIOP_REQUEST_ID_OFFSET + 4 && this->frag_active_ &&
          read_u32_at(msg + GIOP_REQUEST_ID_OFFSET, h.little_endian()) == this->frag_request_id_) {
        this->frag_active_ = false;
        this->frag_.clear();
      }
      return true;
    }
    case GIOP_CLOSE_CONNECTION:
    case GIOP_MESSAGE_ERROR:
      return false;
    default:
      // Reply / LocateReply: never sent to a server.
      giop_write_header_only(tx, h.minor, GIOP_MESSAGE_ERROR);
      return false;
  }
}

bool GiopConnection::process_request_(const GiopHeader &h, const uint8_t *msg, size_t len, std::vector<uint8_t> &tx) {
  GiopRequest req;
  if (!parse_giop_request(msg, len, h, req)) {
    giop_write_header_only(tx, h.minor, GIOP_MESSAGE_ERROR);
    return false;
  }
  this->device_->handle_request(req, this->peer_, tx);
  return true;
}

}  // namespace esphome::tango
