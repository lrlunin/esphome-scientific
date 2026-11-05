#pragma once
// ESPHome component: serves one Tango device over CORBA/GIOP (IIOP) on a TCP port.
//
// Clients reach it without a Tango database:
//   tango://<esp-ip>:<port>/<domain>/<family>/<member>#dbase=no
// or, once the logged IOR has been exported into a Tango database, by name.

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/socket/socket.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

#include "tango_device.h"

namespace esphome::tango {

class TangoServer final : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ---- Configuration (called from the generated code) ---- //
  void set_port(uint16_t port) { this->port_ = port; }
  void set_device_name(const std::string &name) { this->device_.set_name(name); }
  void set_device_class(const std::string &class_name) { this->device_.set_class_name(class_name); }
  void set_server_id(const std::string &server_id) { this->device_.set_server_id(server_id); }
  void set_description(const std::string &description) { this->device_.set_description(description); }
  void set_max_clients(uint8_t max_clients) { this->max_clients_ = max_clients; }
  void set_max_message_size(uint32_t max_message_size) { this->max_message_size_ = max_message_size; }
  void set_idle_timeout(uint32_t idle_timeout_ms) { this->idle_timeout_ms_ = idle_timeout_ms; }

#ifdef USE_SENSOR
  void add_sensor(sensor::Sensor *entity, const std::string &name, const std::string &label,
                  const std::string &description, const std::string &unit, const std::string &format);
#endif
#ifdef USE_BINARY_SENSOR
  void add_binary_sensor(binary_sensor::BinarySensor *entity, const std::string &name, const std::string &label,
                         const std::string &description);
#endif
#ifdef USE_SWITCH
  void add_switch(switch_::Switch *entity, const std::string &name, const std::string &label,
                  const std::string &description);
#endif
#ifdef USE_NUMBER
  void add_number(number::Number *entity, const std::string &name, const std::string &label,
                  const std::string &description, const std::string &unit, const std::string &format);
#endif
#ifdef USE_TEXT_SENSOR
  void add_text_sensor(text_sensor::TextSensor *entity, const std::string &name, const std::string &label,
                       const std::string &description);
#endif
#ifdef USE_BUTTON
  void add_button(button::Button *entity, const std::string &name, const std::string &description);
#endif

  // ---- Runtime API (usable from lambdas) ---- //
  void set_state(DevState state) { this->device_.set_state(state); }
  DevState get_state() const { return this->device_.get_state(); }
  void set_status(const std::string &status) { this->device_.set_status(status); }
  /// Add an attribute or a command defined in C++ (call before or during setup).
  bool add_attribute(const TangoAttribute &attr);
  bool add_command(const TangoCommand &cmd);
  TangoDevice &device() { return this->device_; }
  /// The IOR of the device, empty until the network has an address.
  const std::string &get_ior() const { return this->ior_; }

 protected:
  struct Client {
    std::unique_ptr<socket::Socket> sock;
    std::unique_ptr<GiopConnection> conn;
    std::vector<uint8_t> tx;
    size_t tx_off{0};
    uint32_t last_activity{0};
    bool closing{false};  // Close once tx is flushed.
  };

  void accept_clients_();
  /// Returns false when the client must be dropped now.
  bool service_client_(Client &client, uint32_t now);
  bool flush_(Client &client);
  void close_orderly_(Client &client);
  void update_ior_();

  TangoDevice device_;
  uint16_t port_{10000};
  uint8_t max_clients_{4};
  uint32_t max_message_size_{16384};
  uint32_t idle_timeout_ms_{0};

  std::unique_ptr<socket::ListenSocket> listen_socket_;
  std::vector<std::unique_ptr<Client>> clients_;
  std::vector<std::function<void()>> deferred_;  // Runs in setup(), once entities are configured.
  std::string ior_;
  std::string ior_host_;
  uint32_t last_ior_check_{0};
};

}  // namespace esphome::tango
