// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
//

#ifndef VDA5050_2B_2B_CORE_MESSAGES_MQTT_MODULE_H_
#define VDA5050_2B_2B_CORE_MESSAGES_MQTT_MODULE_H_

#include <mqtt/async_client.h>
#include <vda5050/AgvFactsheet.h>
#include <vda5050/Connection.h>
#include <vda5050/State.h>
#include <vda5050/Visualization.h>

#include <map>
#include <memory>
#include <optional>

#include "vda5050++/agv_description/agv_description.h"
#include "vda5050++/config/mqtt_options.h"
#include "vda5050++/core/module.h"

namespace vda5050pp::core::messages {

///
///\brief The MqttModule is responsible for sending/receiving messages to/from the MQTT broker.
///
class MqttModule final : public vda5050pp::core::Module,
                         public mqtt::callback,
                         public mqtt::iaction_listener {
private:
  ///
  ///\brief An enumeration of the possible states of the MqttModule.
  ///
  enum class State {
    ///\brief the module has been constructed
    k_constructed,
    ///\brief the module has been initialized
    k_initialized,
    ///\brief the module is offline
    k_offline,
    ///\brief the module is online
    k_online,
    ///\brief the module is in an error state
    k_error,
  };
  ///\brief the current state of the module
  State state_ = State::k_constructed;

  std::optional<
      vda5050pp::core::GenericEventManager<vda5050pp::core::events::MessageEvent>::ScopedSubscriber>
      m_subscriber_;
  std::optional<
      vda5050pp::core::GenericEventManager<vda5050pp::core::events::ControlEvent>::ScopedSubscriber>
      c_subscriber_;

  ///\brief the sequence id counter for connection messages
  uint32_t connection_seq_id_ = 0;
  ///\brief the sequence id counter for factsheet messages
  uint32_t factsheet_seq_id_ = 0;
  ///\brief the sequence id counter for state messages
  uint32_t state_seq_id_ = 0;
  ///\brief the sequence id counter for visualization messages
  uint32_t visualization_seq_id_ = 0;

  ///\brief the MQTT client
  std::unique_ptr<mqtt::async_client> mqtt_client_;
  ///\brief the MQTT broker server address string as required by paho
  std::string server_;
  ///\brief the current connection options for paho
  mqtt::connect_options connect_opts_;

  ///\brief the topic string for connection messages (constructed by initialize)
  std::string connection_topic_;
  ///\brief the topic string for factsheet messages (constructed by initialize)
  std::string factsheet_topic_;
  ///\brief the topic string for instant action messages (constructed by initialize)
  std::string instant_actions_topic_;
  ///\brief the topic string for order messages (constructed by initialize)
  std::string order_topic_;
  ///\brief the topic string for state messages (constructed by initialize)
  std::string state_topic_;
  ///\brief the topic string for visualization messages (constructed by initialize)
  std::string visualization_topic_;

  ///\brief the manufacturer, used in each header and each topic path
  std::string manufacturer_;
  ///\brief the serial number, used in each header and each topic path
  std::string serial_number_;

  ///
  ///\brief Fill in a header with information for the connection topic
  ///
  ///\param header The header to write to
  ///
  void fillHeaderConnection(vda5050::HeaderVDA5050 &header);

  ///
  ///\brief Fill in a header with information for the factsheet topic
  ///
  ///\param header The header to write to
  ///
  void fillHeaderFactsheet(vda5050::HeaderVDA5050 &header);

  ///
  ///\brief Fill in a header with information for the state topic
  ///
  ///\param header The header to write to
  ///
  void fillHeaderState(vda5050::HeaderVDA5050 &header);

  ///
  ///\brief Fill in a header with information for the visualization topic
  ///
  ///\param header The header to write to
  ///
  void fillHeaderVisualization(vda5050::HeaderVDA5050 &header);

  ///
  ///\brief Get the last-will, i.e. a connection message with CONNECTIONBROKEN set.
  ///
  ///\return mqtt::will_options The last-will
  ///
  mqtt::will_options getWill();

public:
  ///
  ///\brief Set all relevant member variables with the given MqttOptions from the lib config.
  ///
  /// - connect_opts_
  /// - server_
  /// - *_topic_ will be partially set, i.e. "<iface>/<version>"
  ///
  ///\param opts The library configuration options for the MQTT module
  ///
  void useMqttOptions(const vda5050pp::config::MqttOptions &opts);

  /**
   * This method is invoked when an action fails.
   * @param asyncActionToken
   */
  void on_failure(const mqtt::token &asyncActionToken) override;
  /**
   * This method is invoked when an action has completed successfully.
   * @param asyncActionToken
   */
  void on_success(const mqtt::token &asyncActionToken) override;

  /**
   * This method is called when the client is connected.
   * Note that, in response to an initial connect(), the token from the
   * connect call is also signaled with an on_success(). That occurs just
   * before this is called.
   */
  void connected(const std::string & /*cause*/) override;
  /**
   * This method is called when the connection to the server is lost.
   */
  void connection_lost(const std::string & /*cause*/) override;
  /**
   * This method is called when a message arrives from the server.
   */
  void message_arrived(mqtt::const_message_ptr /*msg*/) override;
  /**
   * Called when delivery for a message has been completed, and all
   * acknowledgments have been received.
   */
  void delivery_complete(mqtt::delivery_token_ptr /*tok*/) override;

  ///
  ///\brief Establish a connection to the MQTT Broker
  ///
  ///\throws VDA5050PPMqttError when the module is not in a state to establish a connection.
  ///
  void connect();

  ///
  ///\brief Disconnect from the MQTT Broker
  ///
  ///\throws VDA5050PPMqttError when the module is not online
  ///
  void disconnect();

  ///
  ///\brief Send a state message (header will be filled in)
  ///
  ///\param state the state to send
  ///
  ///\throws VDA5050PPMqttError when the module is not online
  ///
  void sendState(const vda5050::State &state) const noexcept(false);

  ///
  ///\brief Send a factsheet message (header will be filled in)
  ///
  ///\param factsheet the factsheet to send
  ///
  ///\throws VDA5050PPMqttError when the module is not online
  ///
  void sendFactsheet(const vda5050::AgvFactsheet &state) const noexcept(false);

  ///
  ///\brief Send a visualization message (header will be filled in)
  ///
  ///\param visualization the visualization to send
  ///
  ///\throws VDA5050PPMqttError when the module is not online
  ///
  void sendVisualization(const vda5050::Visualization &visualization) const noexcept(false);

  ///
  ///\brief Send a connection message (header will be filled in)
  ///
  ///\param connection the connection to send
  ///
  ///\throws VDA5050PPMqttError when the module is not online
  ///
  void sendConnection(const vda5050::Connection &connection) const noexcept(false);

  ///
  ///\brief Setup subscribers and fill member variables for the connection
  ///
  ///\param instance The current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribe all subscribers does not disconnect from the MQTT broker
  ///
  ///\param instance The current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a brief description of this module (possibly deprecated)
  ///
  ///\return std::string_view the description
  ///
  std::string_view describe() const override;

  ///
  ///\brief Generate a new SubConfig for the MQTT module
  ///
  ///\return std::shared_ptr<vda5050pp::config::ModuleSubConfig> the sub-config
  ///
  std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateSubConfig() const override;
};

}  // namespace vda5050pp::core::messages

#endif  // VDA5050_2B_2B_CORE_MESSAGES_MQTT_MODULE_H_
