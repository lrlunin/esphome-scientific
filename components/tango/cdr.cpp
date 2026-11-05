#include "cdr.h"

#include <cstring>

namespace esphome::tango {

static inline size_t align_pad(size_t rel, size_t n) { return (n - (rel & (n - 1))) & (n - 1); }

// ============================ Reader ============================ //

void CdrReader::align(size_t n) {
  if (!this->ok_)
    return;
  size_t pad = align_pad(this->pos_ - this->origin_, n);
  if (pad > this->size_ - this->pos_) {
    this->ok_ = false;
    return;
  }
  this->pos_ += pad;
}

bool CdrReader::skip(size_t n) {
  if (!this->ok_ || n > this->size_ - this->pos_) {
    this->ok_ = false;
    return false;
  }
  this->pos_ += n;
  return true;
}

bool CdrReader::take_(size_t align, size_t n, const uint8_t *&p) {
  this->align(align);
  if (!this->ok_ || n > this->size_ - this->pos_) {
    this->ok_ = false;
    return false;
  }
  p = this->data_ + this->pos_;
  this->pos_ += n;
  return true;
}

uint8_t CdrReader::read_octet() {
  const uint8_t *p;
  if (!this->take_(1, 1, p))
    return 0;
  return *p;
}

bool CdrReader::read_bool() { return this->read_octet() != 0; }

uint16_t CdrReader::read_u16_() {
  const uint8_t *p;
  if (!this->take_(2, 2, p))
    return 0;
  if (this->little_)
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
  return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

uint32_t CdrReader::read_u32_() {
  const uint8_t *p;
  if (!this->take_(4, 4, p))
    return 0;
  if (this->little_)
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

uint64_t CdrReader::read_u64_() {
  const uint8_t *p;
  if (!this->take_(8, 8, p))
    return 0;
  uint64_t v = 0;
  if (this->little_) {
    for (int i = 7; i >= 0; i--)
      v = (v << 8) | p[i];
  } else {
    for (int i = 0; i < 8; i++)
      v = (v << 8) | p[i];
  }
  return v;
}

float CdrReader::read_float() {
  uint32_t raw = this->read_u32_();
  float f;
  std::memcpy(&f, &raw, sizeof(f));
  return f;
}

double CdrReader::read_double() {
  uint64_t raw = this->read_u64_();
  double d;
  std::memcpy(&d, &raw, sizeof(d));
  return d;
}

uint32_t CdrReader::read_seq_len(size_t min_elem_size) {
  uint32_t len = this->read_u32_();
  if (!this->ok_)
    return 0;
  if (min_elem_size != 0 && len > (this->size_ - this->pos_) / min_elem_size) {
    this->ok_ = false;
    return 0;
  }
  return len;
}

bool CdrReader::read_string(std::string &out) {
  out.clear();
  uint32_t len = this->read_u32_();
  if (!this->ok_)
    return false;
  if (len == 0)  // Not legal CDR, but some ORBs send it for an empty string.
    return true;
  if (len > this->size_ - this->pos_) {
    this->ok_ = false;
    return false;
  }
  const uint8_t *p = this->data_ + this->pos_;
  if (p[len - 1] != 0) {
    this->ok_ = false;
    return false;
  }
  out.assign(reinterpret_cast<const char *>(p), len - 1);
  this->pos_ += len;
  return true;
}

bool CdrReader::skip_string() {
  uint32_t len = this->read_u32_();
  return this->ok_ && this->skip(len);
}

bool CdrReader::read_octet_seq(std::string &out) {
  out.clear();
  uint32_t len = this->read_seq_len(1);
  if (!this->ok_)
    return false;
  out.assign(reinterpret_cast<const char *>(this->data_ + this->pos_), len);
  this->pos_ += len;
  return true;
}

bool CdrReader::read_encapsulation(CdrReader &out) {
  uint32_t len = this->read_seq_len(1);
  if (!this->ok_)
    return false;
  if (len == 0) {
    this->ok_ = false;
    return false;
  }
  size_t start = this->pos_;
  bool little = this->data_[start] != 0;
  // The sub-reader sees only the encapsulation; its origin is the byte-order flag.
  out = CdrReader(this->data_ + start, len, little, 0, 1);
  this->pos_ += len;
  return true;
}

// ============================ Writer ============================ //

void CdrWriter::align(size_t n) {
  size_t pad = align_pad(this->buf_.size() - this->origin_, n);
  this->buf_.insert(this->buf_.end(), pad, 0);
}

void CdrWriter::write_raw_(const void *v, size_t n) {
  this->align(n);
  const uint8_t *p = static_cast<const uint8_t *>(v);
  this->buf_.insert(this->buf_.end(), p, p + n);
}

void CdrWriter::write_octets(const void *data, size_t len) {
  const uint8_t *p = static_cast<const uint8_t *>(data);
  this->buf_.insert(this->buf_.end(), p, p + len);
}

void CdrWriter::write_string(const char *s) {
  size_t len = std::strlen(s);
  this->write_ulong(static_cast<uint32_t>(len + 1));
  this->write_octets(s, len);
  this->write_octet(0);
}

void CdrWriter::write_string(const std::string &s) {
  // A CDR string cannot carry a NUL octet: cut at the first one.
  size_t len = std::strlen(s.c_str());
  this->write_ulong(static_cast<uint32_t>(len + 1));
  this->write_octets(s.data(), len);
  this->write_octet(0);
}

void CdrWriter::write_octet_seq(const void *data, size_t len) {
  this->write_ulong(static_cast<uint32_t>(len));
  this->write_octets(data, len);
}

void CdrWriter::patch_ulong(size_t pos, uint32_t v) {
  if (pos + 4 <= this->buf_.size())
    std::memcpy(&this->buf_[pos], &v, 4);
}

CdrWriter::Encapsulation CdrWriter::begin_encapsulation() {
  Encapsulation e;
  this->write_ulong(0);  // Length placeholder.
  e.len_pos = this->buf_.size() - 4;
  e.outer_origin = this->origin_;
  this->origin_ = this->buf_.size();
  this->write_bool(HOST_LITTLE_ENDIAN);
  return e;
}

void CdrWriter::end_encapsulation(const Encapsulation &e) {
  uint32_t len = static_cast<uint32_t>(this->buf_.size() - (e.len_pos + 4));
  this->patch_ulong(e.len_pos, len);
  this->origin_ = e.outer_origin;
}

}  // namespace esphome::tango
