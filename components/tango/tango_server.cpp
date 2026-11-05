#include "tango_server.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"
#include "esphome/components/network/util.h"

// ESPHome releases before 2026.10.0 do not provide the macro.
#ifndef ESPHOME_LOG_TAG
#define ESPHOME_LOG_TAG(name, tag) static const char *const name = tag
#endif

namespace esphome::tango {

ESPHOME_LOG_TAG(TAG, "tango");
static constexpr const char *COMPONENT_VERSION = "1.0.0";

// Outgoing octets allowed to pile up for one client before it is dropped.
static constexpr size_t TX_BACKLOG_FACTOR = 4;
// How often the IOR is checked against the current IP address.
static constexpr uint32_t IOR_CHECK_INTERVAL_MS = 5000;

static std::string format_number(float v) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
  return buf;
}

// ======================================================================== //
//                                Setup                                     //
// ======================================================================== //

void TangoServer::setup() {
  // Entity properties (units, number limits...) are configured by the generated
  // code after the add_* calls, so they are read here.
  for (auto &fn : this->deferred_)
    fn();
  this->deferred_.clear();
  this->deferred_.shrink_to_fit();

  this->device_.set_server_host(App.get_name().str());
  this->device_.add_version_info("ESPHome", ESPHOME_VERSION);
  this->device_.add_version_info("esphome-tango", COMPONENT_VERSION);

  this->listen_socket_ = socket::socket_ip_loop_monitored(SOCK_STREAM, 0);
  if (this->listen_socket_ == nullptr) {
    ESP_LOGE(TAG, "Could not create the listening socket");
    this->mark_failed();
    return;
  }
  int enable = 1;
  if (this->listen_socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) != 0) {
    ESP_LOGW(TAG, "Could not set SO_REUSEADDR: errno %d", errno);
  }
  if (this->listen_socket_->setblocking(false) != 0) {
    ESP_LOGE(TAG, "Could not make the listening socket non-blocking: errno %d", errno);
    this->mark_failed();
    return;
  }
  struct sockaddr_storage server;
  socklen_t sl = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&server), sizeof(server), this->port_);
  if (sl == 0) {
    ESP_LOGE(TAG, "Could not set up the listening address: errno %d", errno);
    this->mark_failed();
    return;
  }
  if (this->listen_socket_->bind(reinterpret_cast<struct sockaddr *>(&server), sl) != 0) {
    ESP_LOGE(TAG, "Could not bind port %u: errno %d", this->port_, errno);
    this->mark_failed();
    return;
  }
  if (this->listen_socket_->listen(this->max_clients_) != 0) {
    ESP_LOGE(TAG, "Could not listen: errno %d", errno);
    this->mark_failed();
    return;
  }
  this->clients_.reserve(this->max_clients_);
}

void TangoServer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Tango device server:\n"
                "  Device: %s\n"
                "  Port: %u\n"
                "  Max clients: %u\n"
                "  Max message size: %u\n"
                "  Idle timeout: %u ms",
                this->device_.get_name().c_str(), this->port_, this->max_clients_,
                static_cast<unsigned>(this->max_message_size_), static_cast<unsigned>(this->idle_timeout_ms_));
  if (!this->ior_host_.empty()) {
    ESP_LOGCONFIG(TAG, "  Client URL: tango://%s:%u/%s#dbase=no", this->ior_host_.c_str(), this->port_,
                  this->device_.get_name().c_str());
    ESP_LOGCONFIG(TAG, "  IOR: %s", this->ior_.c_str());
  }
}

void TangoServer::on_shutdown() {
  // Tell the clients to go away cleanly (they reconnect transparently later).
  for (auto &client : this->clients_) {
    this->close_orderly_(*client);
    this->flush_(*client);
    client->sock->close();
  }
  this->clients_.clear();
  if (this->listen_socket_ != nullptr) {
    this->listen_socket_->close();
    this->listen_socket_.reset();
  }
}

bool TangoServer::add_attribute(const TangoAttribute &attr) {
  if (!this->device_.add_attribute(attr)) {
    ESP_LOGE(TAG, "Attribute '%s' rejected (empty, duplicated or unsupported type)", attr.name.c_str());
    return false;
  }
  return true;
}

bool TangoServer::add_command(const TangoCommand &cmd) {
  if (!this->device_.add_command(cmd)) {
    ESP_LOGE(TAG, "Command '%s' rejected (empty, duplicated or unsupported type)", cmd.name.c_str());
    return false;
  }
  return true;
}

// ======================================================================== //
//                             Entity bindings                              //
// ======================================================================== //

#ifdef USE_SENSOR
void TangoServer::add_sensor(sensor::Sensor *entity, const std::string &name, const std::string &label,
                             const std::string &description, const std::string &unit, const std::string &format) {
  TangoAttribute a;
  a.name = name;
  a.data_type = DEV_DOUBLE;
  a.writable = ATTR_READ;
  a.label = label;
  a.description = description;
  a.unit = unit;
  a.format = format;
  a.reader = [entity](AttrReading &r) {
    r.valid = entity->has_state() && !std::isnan(entity->state);
    if (r.valid)
      r.value = TangoValue::make_float(DEV_DOUBLE, entity->state);
  };
  if (!this->add_attribute(a))
    return;
  this->deferred_.emplace_back([this, entity, name, unit, format]() {
    TangoAttribute *attr = this->device_.find_attribute(name);
    if (attr == nullptr)
      return;
    if (unit.empty()) {
      StringRef uom = entity->get_unit_of_measurement_ref();
      if (!uom.empty())
        attr->unit = uom.str();
    }
    if (format.empty()) {
      int8_t decimals = entity->get_accuracy_decimals();
      if (decimals >= 0) {
        char buf[12];
        std::snprintf(buf, sizeof(buf), "%%.%df", static_cast<int>(decimals));
        attr->format = buf;
      }
    }
  });
}
#endif

#ifdef USE_BINARY_SENSOR
void TangoServer::add_binary_sensor(binary_sensor::BinarySensor *entity, const std::string &name,
                                    const std::string &label, const std::string &description) {
  TangoAttribute a;
  a.name = name;
  a.data_type = DEV_BOOLEAN;
  a.writable = ATTR_READ;
  a.label = label;
  a.description = description;
  a.reader = [entity](AttrReading &r) {
    r.valid = entity->has_state();
    if (r.valid)
      r.value = TangoValue::make_bool(entity->state);
  };
  this->add_attribute(a);
}
#endif

#ifdef USE_SWITCH
void TangoServer::add_switch(switch_::Switch *entity, const std::string &name, const std::string &label,
                             const std::string &description) {
  TangoAttribute a;
  a.name = name;
  a.data_type = DEV_BOOLEAN;
  a.writable = ATTR_READ_WRITE;
  a.label = label;
  a.description = description;
  a.track_set_point = false;
  a.reader = [entity](AttrReading &r) {
    r.valid = true;
    r.value = TangoValue::make_bool(entity->state);
  };
  a.writer = [entity](const TangoValue &v, DevErrorList &) {
    if (v.b) {
      entity->turn_on();
    } else {
      entity->turn_off();
    }
  };
  this->add_attribute(a);
}
#endif

#ifdef USE_NUMBER
void TangoServer::add_number(number::Number *entity, const std::string &name, const std::string &label,
                             const std::string &description, const std::string &unit, const std::string &format) {
  TangoAttribute a;
  a.name = name;
  a.data_type = DEV_DOUBLE;
  a.writable = ATTR_READ_WRITE;
  a.label = label;
  a.description = description;
  a.unit = unit;
  a.format = format;
  a.track_set_point = false;
  a.reader = [entity](AttrReading &r) {
    r.valid = entity->has_state() && !std::isnan(entity->state);
    if (r.valid)
      r.value = TangoValue::make_float(DEV_DOUBLE, entity->state);
  };
  a.writer = [entity](const TangoValue &v, DevErrorList &) {
    auto call = entity->make_call();
    call.set_value(static_cast<float>(v.d));
    call.perform();
  };
  if (!this->add_attribute(a))
    return;
  this->deferred_.emplace_back([this, entity, name, unit]() {
    TangoAttribute *attr = this->device_.find_attribute(name);
    if (attr == nullptr)
      return;
    float lo = entity->traits.get_min_value();
    float hi = entity->traits.get_max_value();
    if (std::isfinite(lo))
      attr->min_value = format_number(lo);
    if (std::isfinite(hi))
      attr->max_value = format_number(hi);
    if (unit.empty()) {
      StringRef uom = entity->get_unit_of_measurement_ref();
      if (!uom.empty())
        attr->unit = uom.str();
    }
  });
}
#endif

#ifdef USE_TEXT_SENSOR
void TangoServer::add_text_sensor(text_sensor::TextSensor *entity, const std::string &name, const std::string &label,
                                  const std::string &description) {
  TangoAttribute a;
  a.name = name;
  a.data_type = DEV_STRING;
  a.writable = ATTR_READ;
  a.label = label;
  a.description = description;
  a.reader = [entity](AttrReading &r) {
    r.valid = entity->has_state();
    if (r.valid)
      r.value = TangoValue::make_string(entity->state);
  };
  this->add_attribute(a);
}
#endif

#ifdef USE_BUTTON
void TangoServer::add_button(button::Button *entity, const std::string &name, const std::string &description) {
  TangoCommand c;
  c.name = name;
  c.in_type = DEV_VOID;
  c.out_type = DEV_VOID;
  c.in_desc = description;
  c.handler = [entity](const TangoValue &, TangoValue &, DevErrorList &) { entity->press(); };
  this->add_command(c);
}
#endif

// ======================================================================== //
//                                 Loop                                     //
// ======================================================================== //

void TangoServer::loop() {
  if (this->listen_socket_ == nullptr)
    return;
  const uint32_t now = App.get_loop_component_start_time();

  if (this->listen_socket_->ready())
    this->accept_clients_();

  for (size_t i = 0; i < this->clients_.size();) {
    if (this->service_client_(*this->clients_[i], now)) {
      i++;
    } else {
      ESP_LOGD(TAG, "Client %s disconnected", this->clients_[i]->conn->peer().c_str());
      this->clients_[i]->sock->close();
      this->clients_.erase(this->clients_.begin() + i);
    }
  }

  if (this->ior_.empty() || now - this->last_ior_check_ >= IOR_CHECK_INTERVAL_MS) {
    this->last_ior_check_ = now;
    this->update_ior_();
  }
}

void TangoServer::accept_clients_() {
  while (true) {
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    auto sock =
        this->listen_socket_->accept_loop_monitored(reinterpret_cast<struct sockaddr *>(&source_addr), &addr_len);
    if (!sock)
      break;

    char peer[socket::SOCKADDR_STR_LEN];
    sock->getpeername_to(peer);

    if (this->clients_.size() >= this->max_clients_) {
      // CORBA clients keep idle connections open: make room by releasing the
      // least recently used idle one (GIOP CloseConnection, the ORB reconnects
      // transparently when it needs to).
      size_t victim = this->clients_.size();
      for (size_t i = 0; i < this->clients_.size(); i++) {
        Client &c = *this->clients_[i];
        if (c.conn->busy() || c.tx_off < c.tx.size())
          continue;
        if (victim == this->clients_.size() || c.last_activity < this->clients_[victim]->last_activity)  // NOLINT
          victim = i;
      }
      if (victim == this->clients_.size()) {
        ESP_LOGW(TAG, "Max clients (%u) busy, rejecting %s", this->max_clients_, peer);
        sock->close();
        continue;
      }
      Client &old = *this->clients_[victim];
      ESP_LOGD(TAG, "Releasing idle client %s for %s", old.conn->peer().c_str(), peer);
      this->close_orderly_(old);
      this->flush_(old);
      old.sock->close();
      this->clients_.erase(this->clients_.begin() + victim);
    }

    if (sock->setblocking(false) != 0) {
      ESP_LOGW(TAG, "Could not make the socket of %s non-blocking", peer);
      sock->close();
      continue;
    }
    int enable = 1;
    sock->setsockopt(IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(int));

    auto client = std::make_unique<Client>();
    client->sock = std::move(sock);
    client->conn = std::make_unique<GiopConnection>(&this->device_, this->max_message_size_);
    client->conn->set_peer(peer);
    client->last_activity = App.get_loop_component_start_time();
    ESP_LOGD(TAG, "Client %s connected", peer);
    this->clients_.push_back(std::move(client));
  }
}

bool TangoServer::service_client_(Client &client, uint32_t now) {
  if (!this->flush_(client))
    return false;

  // The socket contract requires reading until it would block once ready().
  if (!client.closing && client.sock->ready()) {
    uint8_t buf[512];
    while (true) {
      ssize_t n = client.sock->read(buf, sizeof(buf));
      if (n > 0) {
        client.last_activity = now;
        if (!client.conn->feed(buf, static_cast<size_t>(n), client.tx)) {
          client.closing = true;
          break;
        }
        if (client.tx.size() - client.tx_off > TX_BACKLOG_FACTOR * this->max_message_size_) {
          ESP_LOGW(TAG, "Client %s does not read its replies, dropping it", client.conn->peer().c_str());
          return false;
        }
        continue;
      }
      if (n == 0)
        return false;  // Peer closed the connection.
      if (errno == EWOULDBLOCK || errno == EAGAIN)
        break;
      return false;
    }
  }

  if (!this->flush_(client))
    return false;
  if (client.closing)
    return client.tx_off < client.tx.size();  // Keep until the last octets are out.

  if (this->idle_timeout_ms_ != 0 && now - client.last_activity > this->idle_timeout_ms_ && !client.conn->busy() &&
      client.tx.empty()) {
    ESP_LOGD(TAG, "Client %s idle, closing", client.conn->peer().c_str());
    this->close_orderly_(client);
    this->flush_(client);
    return client.tx_off < client.tx.size();
  }
  return true;
}

bool TangoServer::flush_(Client &client) {
  while (client.tx_off < client.tx.size()) {
    ssize_t n = client.sock->write(client.tx.data() + client.tx_off, client.tx.size() - client.tx_off);
    if (n > 0) {
      client.tx_off += static_cast<size_t>(n);
      continue;
    }
    if (n == 0 || errno == EWOULDBLOCK || errno == EAGAIN)
      break;  // TCP send buffer full: retry on the next loop.
    return false;
  }
  if (client.tx_off >= client.tx.size()) {
    client.tx.clear();
    client.tx_off = 0;
  }
  return true;
}

void TangoServer::close_orderly_(Client &client) {
  if (client.closing)
    return;
  giop_write_header_only(client.tx, client.conn->last_minor(), GIOP_CLOSE_CONNECTION);
  client.closing = true;
}

void TangoServer::update_ior_() {
  if (!network::is_connected())
    return;
  // Only str_to() and is_ip4() exist on every platform's IPAddress: an unset
  // address is recognised by its text form.
  auto usable = [](const char *s) {
    return s != nullptr && s[0] != '\0' && std::strcmp(s, "0.0.0.0") != 0 && std::strcmp(s, "::") != 0;
  };
  char buf[network::IP_ADDRESS_BUFFER_SIZE];
  std::string host;
  network::IPAddresses addresses = network::get_ip_addresses();
  for (int pass = 0; pass < 2 && host.empty(); pass++) {
    for (auto &ip : addresses) {
      if (pass == 0 && !ip.is_ip4())
        continue;  // Prefer IPv4: every Tango client handles it.
      buf[0] = '\0';
      const char *s = ip.str_to(buf);
      if (usable(s)) {
        host = s;
        break;
      }
    }
  }
  if (host.empty() || this->ior_host_ == host)
    return;
  this->ior_host_ = host;
  this->ior_ = this->device_.build_ior(this->ior_host_, this->port_);
  ESP_LOGI(TAG, "Serving tango://%s:%u/%s#dbase=no", this->ior_host_.c_str(), this->port_,
           this->device_.get_name().c_str());
  ESP_LOGI(TAG, "IOR: %s", this->ior_.c_str());
}

}  // namespace esphome::tango
