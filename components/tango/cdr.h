#pragma once
// CORBA Common Data Representation (CDR) encoder / decoder.
//
// Specification: OMG CORBA 3.x, Part 2 (Interoperability), chapter 9.3.
//
// * Every primitive is aligned on its own size (max 8), relative to an "origin":
//   the first octet of the GIOP message, or the first octet (the byte-order flag)
//   of an encapsulation.
// * The reader accepts both byte orders, the writer always emits the host order
//   (GIOP tells the peer which one through the header flag / encapsulation flag).
//
// This file does not depend on ESPHome and does not use C++ exceptions: the reader
// latches an error flag that callers check with ok().

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace esphome::tango {

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
static constexpr bool HOST_LITTLE_ENDIAN = false;
#else
static constexpr bool HOST_LITTLE_ENDIAN = true;
#endif

class CdrReader {
 public:
  CdrReader() = default;
  /// `data`/`size` is the whole stream, `origin` is the offset alignment is computed
  /// from, `pos` the offset reading starts at.
  CdrReader(const uint8_t *data, size_t size, bool little_endian, size_t origin = 0, size_t pos = 0)
      : data_(data), size_(size), origin_(origin), pos_(pos), little_(little_endian), ok_(pos <= size) {}

  bool ok() const { return this->ok_; }
  void fail() { this->ok_ = false; }
  bool little_endian() const { return this->little_; }
  size_t position() const { return this->pos_; }
  size_t size() const { return this->size_; }
  size_t remaining() const { return this->ok_ && this->pos_ <= this->size_ ? this->size_ - this->pos_ : 0; }
  const uint8_t *data() const { return this->data_; }

  void align(size_t n);
  bool skip(size_t n);

  uint8_t read_octet();
  bool read_bool();
  int16_t read_short() { return static_cast<int16_t>(this->read_u16_()); }
  uint16_t read_ushort() { return this->read_u16_(); }
  int32_t read_long() { return static_cast<int32_t>(this->read_u32_()); }
  uint32_t read_ulong() { return this->read_u32_(); }
  int64_t read_longlong() { return static_cast<int64_t>(this->read_u64_()); }
  uint64_t read_ulonglong() { return this->read_u64_(); }
  float read_float();
  double read_double();

  /// Read a CDR string (ulong length including the NUL, then the octets). The
  /// octets are returned as-is: Tango strings are ISO-8859-1 / UTF-8 bytes.
  bool read_string(std::string &out);
  /// Skip a CDR string.
  bool skip_string();
  /// Read a sequence length and check that `len * min_elem_size` octets can follow.
  uint32_t read_seq_len(size_t min_elem_size);
  /// Read a sequence<octet> into `out`.
  bool read_octet_seq(std::string &out);
  /// Open an encapsulation (sequence<octet> whose first octet is the byte order),
  /// returning a reader positioned right after the byte-order flag.
  bool read_encapsulation(CdrReader &out);

 private:
  uint16_t read_u16_();
  uint32_t read_u32_();
  uint64_t read_u64_();
  bool take_(size_t align, size_t n, const uint8_t *&p);

  const uint8_t *data_{nullptr};
  size_t size_{0};
  size_t origin_{0};
  size_t pos_{0};
  bool little_{true};
  bool ok_{false};
};

class CdrWriter {
 public:
  struct Encapsulation {
    size_t len_pos;
    size_t outer_origin;
  };

  CdrWriter() { this->buf_.reserve(256); }

  std::vector<uint8_t> &buffer() { return this->buf_; }
  const std::vector<uint8_t> &buffer() const { return this->buf_; }
  size_t position() const { return this->buf_.size(); }
  size_t origin() const { return this->origin_; }
  void truncate(size_t pos) {
    if (pos < this->buf_.size())
      this->buf_.resize(pos);
  }
  void clear() {
    this->buf_.clear();
    this->origin_ = 0;
  }

  void align(size_t n);
  void write_octet(uint8_t v) { this->buf_.push_back(v); }
  void write_bool(bool v) { this->buf_.push_back(v ? 1 : 0); }
  void write_short(int16_t v) { this->write_raw_(&v, sizeof(v)); }
  void write_ushort(uint16_t v) { this->write_raw_(&v, sizeof(v)); }
  void write_long(int32_t v) { this->write_raw_(&v, sizeof(v)); }
  void write_ulong(uint32_t v) { this->write_raw_(&v, sizeof(v)); }
  void write_longlong(int64_t v) { this->write_raw_(&v, sizeof(v)); }
  void write_ulonglong(uint64_t v) { this->write_raw_(&v, sizeof(v)); }
  void write_float(float v) { this->write_raw_(&v, sizeof(v)); }
  void write_double(double v) { this->write_raw_(&v, sizeof(v)); }
  /// Raw octets, no alignment, no length.
  void write_octets(const void *data, size_t len);
  void write_string(const char *s);
  void write_string(const std::string &s);
  void write_octet_seq(const void *data, size_t len);
  /// Overwrite an ulong (host order) previously written at `pos`.
  void patch_ulong(size_t pos, uint32_t v);

  /// Start an encapsulation: writes the length placeholder and the byte-order flag,
  /// and makes alignment relative to the encapsulation.
  Encapsulation begin_encapsulation();
  void end_encapsulation(const Encapsulation &e);

 private:
  void write_raw_(const void *v, size_t n);

  std::vector<uint8_t> buf_;
  size_t origin_{0};
};

}  // namespace esphome::tango
