#include "giop.h"

#include <cstring>

namespace esphome::tango {

bool parse_giop_header(const uint8_t *p, GiopHeader &h) {
  if (std::memcmp(p, "GIOP", 4) != 0)
    return false;
  h.major = p[4];
  h.minor = p[5];
  h.flags = p[6];
  h.type = p[7];
  // GIOP 1.0 has a boolean byte_order at offset 6, 1.1+ a flags octet whose bit 0
  // is the byte order: both read the same way.
  if (h.flags & GIOP_FLAG_LITTLE_ENDIAN) {
    h.size = uint32_t(p[8]) | (uint32_t(p[9]) << 8) | (uint32_t(p[10]) << 16) | (uint32_t(p[11]) << 24);
  } else {
    h.size = (uint32_t(p[8]) << 24) | (uint32_t(p[9]) << 16) | (uint32_t(p[10]) << 8) | uint32_t(p[11]);
  }
  return true;
}

// IOP::ServiceContextList = sequence<struct { ulong context_id; sequence<octet> context_data; }>
static void skip_service_contexts(CdrReader &r) {
  uint32_t n = r.read_seq_len(8);
  for (uint32_t i = 0; i < n && r.ok(); i++) {
    r.read_ulong();
    uint32_t len = r.read_seq_len(1);
    r.skip(len);
  }
}

// IIOP::ProfileBody_1_x, read from its encapsulation: only the object key is kept.
static bool parse_iiop_profile_key(CdrReader &enc, std::string &key) {
  enc.read_octet();   // iiop_version.major
  enc.read_octet();   // iiop_version.minor
  enc.skip_string();  // host
  enc.read_ushort();  // port
  enc.read_octet_seq(key);
  return enc.ok();
}

// GIOP 1.2 TargetAddress union.
static void read_target_address(CdrReader &r, GiopRequest &out) {
  int16_t disposition = r.read_short();
  if (!r.ok())
    return;
  switch (disposition) {
    case ADDR_KEY:
      if (r.read_octet_seq(out.object_key))
        out.key_resolved = true;
      break;
    case ADDR_PROFILE: {
      // IOP::TaggedProfile { ulong tag; sequence<octet> profile_data; }
      uint32_t tag = r.read_ulong();
      CdrReader enc;
      if (!r.read_encapsulation(enc))
        break;
      if (tag == IOP_TAG_INTERNET_IOP && parse_iiop_profile_key(enc, out.object_key))
        out.key_resolved = true;
      break;
    }
    case ADDR_REFERENCE: {
      // IORAddressingInfo { ulong selected_profile_index; IOR ior; }
      // IOR { string type_id; sequence<TaggedProfile> profiles; }
      uint32_t index = r.read_ulong();
      r.skip_string();
      uint32_t n = r.read_seq_len(8);
      for (uint32_t i = 0; i < n && r.ok(); i++) {
        uint32_t tag = r.read_ulong();
        CdrReader enc;
        if (!r.read_encapsulation(enc))
          break;
        if (i == index && tag == IOP_TAG_INTERNET_IOP && parse_iiop_profile_key(enc, out.object_key))
          out.key_resolved = true;
      }
      break;
    }
    default:
      r.fail();
      break;
  }
}

bool parse_giop_request(const uint8_t *msg, size_t len, const GiopHeader &h, GiopRequest &out) {
  out = GiopRequest();
  out.minor = h.minor;
  out.little = h.little_endian();
  out.msg = msg;
  out.msg_len = len;

  CdrReader r(msg, len, out.little, 0, GIOP_HEADER_LEN);

  if (h.minor <= 1) {
    // RequestHeader_1_0 / RequestHeader_1_1
    skip_service_contexts(r);
    out.request_id = r.read_ulong();
    out.response_expected = r.read_bool();
    if (h.minor == 1)
      r.skip(3);  // octet reserved[3]
    if (r.read_octet_seq(out.object_key))
      out.key_resolved = true;
    r.read_string(out.operation);
    std::string principal;
    r.read_octet_seq(principal);  // Principal requesting_principal
    if (!r.ok())
      return false;
    out.body_offset = r.position();
  } else {
    // RequestHeader_1_2
    out.request_id = r.read_ulong();
    uint8_t response_flags = r.read_octet();
    r.skip(3);  // octet reserved[3]
    out.response_expected = (response_flags & GIOP_RESPONSE_EXPECTED) != 0;
    read_target_address(r, out);
    r.read_string(out.operation);
    skip_service_contexts(r);
    if (!r.ok())
      return false;
    // The 1.2 body is aligned on 8 octets from the message start, when there is one.
    size_t pos = r.position();
    if (pos < len) {
      size_t pad = (8 - (pos & 7)) & 7;
      pos = pos + pad > len ? len : pos + pad;
    }
    out.body_offset = pos;
  }
  return !out.operation.empty();
}

bool parse_giop_locate_request(const uint8_t *msg, size_t len, const GiopHeader &h, GiopRequest &out) {
  out = GiopRequest();
  out.minor = h.minor;
  out.little = h.little_endian();
  out.msg = msg;
  out.msg_len = len;
  CdrReader r(msg, len, out.little, 0, GIOP_HEADER_LEN);
  out.request_id = r.read_ulong();
  if (h.minor <= 1) {
    if (r.read_octet_seq(out.object_key))
      out.key_resolved = true;
  } else {
    read_target_address(r, out);
  }
  out.body_offset = len;
  return r.ok();
}

// ============================ Replies ============================ //

void GiopReplyBuilder::header_(uint8_t minor, uint8_t type) {
  this->w_.clear();
  const uint8_t head[8] = {
      'G', 'I', 'O', 'P', 1, minor, static_cast<uint8_t>(HOST_LITTLE_ENDIAN ? GIOP_FLAG_LITTLE_ENDIAN : 0), type};
  this->w_.write_octets(head, sizeof(head));
  this->w_.write_ulong(0);  // Message size, patched by finish().
}

void GiopReplyBuilder::begin(uint8_t minor, uint32_t request_id, uint32_t status) {
  this->header_(minor, GIOP_REPLY);
  if (minor <= 1) {
    // ReplyHeader_1_0 { ServiceContextList; ulong request_id; ReplyStatusType reply_status; }
    this->w_.write_ulong(0);
    this->w_.write_ulong(request_id);
    this->w_.write_ulong(status);
    this->header_end_ = this->w_.position();
    this->body_start_ = this->header_end_;
  } else {
    // ReplyHeader_1_2 { ulong request_id; ReplyStatusType_1_2 reply_status; ServiceContextList; }
    this->w_.write_ulong(request_id);
    this->w_.write_ulong(status);
    this->w_.write_ulong(0);
    this->header_end_ = this->w_.position();
    this->w_.align(8);
    this->body_start_ = this->w_.position();
  }
}

void GiopReplyBuilder::begin_locate(uint8_t minor, uint32_t request_id, uint32_t status) {
  this->header_(minor, GIOP_LOCATE_REPLY);
  // LocateReplyHeader { ulong request_id; LocateStatusType locate_status; }
  this->w_.write_ulong(request_id);
  this->w_.write_ulong(status);
  this->header_end_ = this->w_.position();
  if (minor >= 2)
    this->w_.align(8);
  this->body_start_ = this->w_.position();
}

void GiopReplyBuilder::finish() {
  if (this->w_.position() == this->body_start_)
    this->w_.truncate(this->header_end_);  // No body: no body padding either.
  this->w_.patch_ulong(8, static_cast<uint32_t>(this->w_.position() - GIOP_HEADER_LEN));
}

void GiopReplyBuilder::system_exception(uint8_t minor, uint32_t request_id, const char *repo_id, uint32_t minor_code,
                                        CompletionStatus completed) {
  this->begin(minor, request_id, REPLY_SYSTEM_EXCEPTION);
  // SystemExceptionReplyBody { string exception_id; ulong minor_code_value; ulong completion_status; }
  this->w_.write_string(repo_id);
  this->w_.write_ulong(minor_code);
  this->w_.write_ulong(completed);
  this->finish();
}

void giop_write_header_only(std::vector<uint8_t> &out, uint8_t minor, uint8_t type) {
  const uint8_t head[12] = {
      'G',  'I', 'O', 'P', 1, minor, static_cast<uint8_t>(HOST_LITTLE_ENDIAN ? GIOP_FLAG_LITTLE_ENDIAN : 0),
      type, 0,   0,   0,   0};
  out.insert(out.end(), head, head + sizeof(head));
}

}  // namespace esphome::tango
