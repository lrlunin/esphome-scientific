#pragma once
// General Inter-ORB Protocol (GIOP 1.0 / 1.1 / 1.2) over TCP (IIOP), server side.
//
// Specification: OMG CORBA 3.x, Part 2 (Interoperability), chapter 9.4.
// Wire layout cross-checked with rsTango (src/corba/orb/giop.rs, server.rs).
//
// Only what a Tango device server needs is implemented: Request, Reply,
// CancelRequest, LocateRequest, LocateReply, CloseConnection, MessageError and
// Fragment (1.1 and 1.2 reassembly).

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "cdr.h"

namespace esphome::tango {

static constexpr size_t GIOP_HEADER_LEN = 12;
/// Offset of the request id in a GIOP 1.2 Request, Fragment and CancelRequest (first header member).
static constexpr size_t GIOP_REQUEST_ID_OFFSET = GIOP_HEADER_LEN;
/// GIOP header flags (GIOP 1.0: only the byte order, as a boolean).
static constexpr uint8_t GIOP_FLAG_LITTLE_ENDIAN = 0x01;
static constexpr uint8_t GIOP_FLAG_FRAGMENT = 0x02;  // 1.1+
/// GIOP 1.2 RequestHeader::response_flags bit: a reply is expected.
static constexpr uint8_t GIOP_RESPONSE_EXPECTED = 0x01;
/// IOP::ProfileId of an IIOP profile.
static constexpr uint32_t IOP_TAG_INTERNET_IOP = 0;

enum GiopMsgType : uint8_t {
  GIOP_REQUEST = 0,
  GIOP_REPLY = 1,
  GIOP_CANCEL_REQUEST = 2,
  GIOP_LOCATE_REQUEST = 3,
  GIOP_LOCATE_REPLY = 4,
  GIOP_CLOSE_CONNECTION = 5,
  GIOP_MESSAGE_ERROR = 6,
  GIOP_FRAGMENT = 7,
};

enum GiopReplyStatus : uint32_t {
  REPLY_NO_EXCEPTION = 0,
  REPLY_USER_EXCEPTION = 1,
  REPLY_SYSTEM_EXCEPTION = 2,
  REPLY_LOCATION_FORWARD = 3,
  REPLY_LOCATION_FORWARD_PERM = 4,  // 1.2+
  REPLY_NEEDS_ADDRESSING_MODE = 5,  // 1.2+
};

enum GiopLocateStatus : uint32_t {
  LOCATE_UNKNOWN_OBJECT = 0,
  LOCATE_OBJECT_HERE = 1,
  LOCATE_OBJECT_FORWARD = 2,
  LOCATE_OBJECT_FORWARD_PERM = 3,        // 1.2+
  LOCATE_LOC_SYSTEM_EXCEPTION = 4,       // 1.2+
  LOCATE_LOC_NEEDS_ADDRESSING_MODE = 5,  // 1.2+
};

enum CompletionStatus : uint32_t {
  COMPLETED_YES = 0,
  COMPLETED_NO = 1,
  COMPLETED_MAYBE = 2,
};

// Standard system exception repository ids used by this server.
namespace sysex {
static constexpr const char *UNKNOWN = "IDL:omg.org/CORBA/UNKNOWN:1.0";
static constexpr const char *BAD_PARAM = "IDL:omg.org/CORBA/BAD_PARAM:1.0";
static constexpr const char *NO_MEMORY = "IDL:omg.org/CORBA/NO_MEMORY:1.0";
static constexpr const char *IMP_LIMIT = "IDL:omg.org/CORBA/IMP_LIMIT:1.0";
static constexpr const char *MARSHAL = "IDL:omg.org/CORBA/MARSHAL:1.0";
static constexpr const char *NO_IMPLEMENT = "IDL:omg.org/CORBA/NO_IMPLEMENT:1.0";
static constexpr const char *BAD_OPERATION = "IDL:omg.org/CORBA/BAD_OPERATION:1.0";
static constexpr const char *OBJECT_NOT_EXIST = "IDL:omg.org/CORBA/OBJECT_NOT_EXIST:1.0";
static constexpr const char *INTERNAL = "IDL:omg.org/CORBA/INTERNAL:1.0";
}  // namespace sysex

// GIOP 1.2 TargetAddress discriminants.
enum GiopAddressing : int16_t {
  ADDR_KEY = 0,
  ADDR_PROFILE = 1,
  ADDR_REFERENCE = 2,
};

struct GiopHeader {
  uint8_t major{0};
  uint8_t minor{0};
  uint8_t flags{0};
  uint8_t type{0};
  uint32_t size{0};  // Octets following the 12 octet header.
  bool little_endian() const { return (this->flags & GIOP_FLAG_LITTLE_ENDIAN) != 0; }
  bool fragment_follows() const { return this->minor >= 1 && (this->flags & GIOP_FLAG_FRAGMENT) != 0; }
};

/// Parse the fixed 12 octet header. Returns false if the magic is not "GIOP".
bool parse_giop_header(const uint8_t *p, GiopHeader &h);

/// A parsed Request (any of 1.0, 1.1, 1.2) or LocateRequest.
struct GiopRequest {
  uint8_t minor{2};
  bool little{true};
  uint32_t request_id{0};
  bool response_expected{true};
  bool key_resolved{false};  // false when the target could not be turned into an object key
  std::string object_key;
  std::string operation;
  const uint8_t *msg{nullptr};  // Whole message, header included.
  size_t msg_len{0};
  size_t body_offset{0};

  /// Reader positioned on the first octet of the body, aligned on the message start.
  CdrReader body() const { return CdrReader(this->msg, this->msg_len, this->little, 0, this->body_offset); }
};

/// Parse a Request message (`msg` holds `len` octets, header included).
bool parse_giop_request(const uint8_t *msg, size_t len, const GiopHeader &h, GiopRequest &out);
/// Parse a LocateRequest message.
bool parse_giop_locate_request(const uint8_t *msg, size_t len, const GiopHeader &h, GiopRequest &out);

/// Builds one reply message into its own writer (alignment origin = message start).
class GiopReplyBuilder {
 public:
  /// Start a Reply; the body can be written into `writer()` right after.
  void begin(uint8_t minor, uint32_t request_id, uint32_t status);
  /// Start a LocateReply.
  void begin_locate(uint8_t minor, uint32_t request_id, uint32_t status);
  /// Patch the message size. Strips the 1.2 body padding when no body was written.
  void finish();
  CdrWriter &writer() { return this->w_; }
  const std::vector<uint8_t> &bytes() const { return this->w_.buffer(); }

  /// Convenience: a complete SYSTEM_EXCEPTION reply.
  void system_exception(uint8_t minor, uint32_t request_id, const char *repo_id, uint32_t minor_code,
                        CompletionStatus completed);

 private:
  void header_(uint8_t minor, uint8_t type);
  CdrWriter w_;
  size_t header_end_{0};
  size_t body_start_{0};
};

/// Append a complete header-only message (CloseConnection / MessageError).
void giop_write_header_only(std::vector<uint8_t> &out, uint8_t minor, uint8_t type);

}  // namespace esphome::tango
