//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_INSTANCE_H_
#define PRIVATE_VDA5050_2B_2B_CORE_INSTANCE_H_

#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>

#include "vda5050++/config.h"
#include "vda5050++/core/action_event_manager.h"
#include "vda5050++/core/action_status_manager.h"
#include "vda5050++/core/events/control_event.h"
#include "vda5050++/core/events/event_manager_options.h"
#include "vda5050++/core/events/factsheet_event.h"
#include "vda5050++/core/events/interpreter_event.h"
#include "vda5050++/core/events/message_event.h"
#include "vda5050++/core/events/order_event.h"
#include "vda5050++/core/events/state_event.h"
#include "vda5050++/core/events/validation_event.h"
#include "vda5050++/core/generic_event_manager.h"
#include "vda5050++/core/navigation_event_manager.h"
#include "vda5050++/core/navigation_status_manager.h"
#include "vda5050++/core/order_event_manager.h"
#include "vda5050++/core/query_event_manager.h"
#include "vda5050++/core/state/map_manager.h"
#include "vda5050++/core/state/order_manager.h"
#include "vda5050++/core/state/status_manager.h"
#include "vda5050++/core/status_event_manager.h"
#include "vda5050++/handler/base_action_handler.h"
#include "vda5050++/handler/base_navigation_handler.h"
#include "vda5050++/handler/base_query_handler.h"

namespace vda5050pp::core {

namespace module_keys {
static constexpr std::string_view k_action_event_handler_key = "ActionEventHandler";
static constexpr std::string_view k_distance_node_handler = "DistanceNodeHandler";
static constexpr std::string_view k_factsheet_event_handler_key = "Factsheet";
static constexpr std::string_view k_interpreter_event_handler_key = "InterpreterEventHandler";
static constexpr std::string_view k_navigation_event_handler_key = "NavigationEventHandler";
static constexpr std::string_view k_message_event_handler_key = "MessageEventHandler";
static constexpr std::string_view k_mqtt_key = "Mqtt";
static constexpr std::string_view k_node_reached_handler_key = "NodeReachedHandler";
static constexpr std::string_view k_order_event_handler_key = "OrderEventHandler";
static constexpr std::string_view k_query_event_handler_key = "QueryEventHandler";
static constexpr std::string_view k_state_event_handler_key = "StateEventHandler";
static constexpr std::string_view k_state_update_timer_key = "StateUpdateTimer";
static constexpr std::string_view k_validation_event_handler_key = "ValidationEventHandler";
static constexpr std::string_view k_visualization_timer_key = "VisualizationTimer";
}  // namespace module_keys

class Module;

///
///\brief The Instance class holds all components of the library. It is a singleton class,
/// which will be constructed when initializing the library.
///
class Instance {
private:
  ///\brief The singleton instance.
  static std::shared_ptr<Instance> instance_;
  ///\brief a mutex protecting the singleton instance.
  static std::shared_mutex instance_mutex_;

  ///\brief The config used to initialize the library.
  vda5050pp::Config config_;

  ///\brief the current event manager options
  vda5050pp::core::events::EventManagerOptions event_manager_options_;

  ///\brief The event manager for action events
  ActionEventManager action_event_manager_;
  ///\brief The event manager for action status events
  ActionStatusManager action_status_manager_;
  ///\brief The event manager for navigation events
  NavigationEventManager navigation_event_manager_;
  ///\brief The event manager for navigation status events
  NavigationStatusManager navigation_status_manager_;
  ///\brief The event manager for status events
  StatusEventManager status_event_manager_;
  ///\brief The event manager for query events
  QueryEventManager query_event_manager_;
  ///\brief The event manager for agv order events
  OrderEventManager agv_order_event_manager_;

  ///\brief The event manager for internal control events
  GenericEventManager<vda5050pp::core::events::ControlEvent> control_event_manager_;
  ///\brief The event manager for internal factsheet events
  GenericEventManager<vda5050pp::core::events::FactsheetEvent> factsheet_event_manager_;
  ///\brief The event manager for internal interpreter events
  GenericEventManager<vda5050pp::core::events::InterpreterEvent> interpreter_event_manager_;
  ///\brief The event manager for internal message events
  GenericEventManager<vda5050pp::core::events::MessageEvent> message_event_manager_;
  ///\brief The event manager for internal order events
  GenericEventManager<vda5050pp::core::events::OrderEvent> order_event_manager_;
  ///\brief The event manager for internal state events
  GenericEventManager<vda5050pp::core::events::StateEvent> state_event_manager_;
  ///\brief The event manager for internal validation events
  GenericEventManager<vda5050pp::core::events::ValidationEvent> validation_event_manager_;

  ///\brief The module registry, which will be initialized/deinitialized by the instance.
  static std::map<std::string, std::shared_ptr<vda5050pp::core::Module>, std::less<>> modules_;
  ///\brief Protects the module registry
  static std::shared_mutex modules_mutex_;

  ///\brief All registered action handlers
  std::set<std::shared_ptr<vda5050pp::handler::BaseActionHandler>> action_handler_;
  ///\brief The registered navigation handler
  std::shared_ptr<vda5050pp::handler::BaseNavigationHandler> navigation_handler_;
  ///\brief The registered query handler
  std::shared_ptr<vda5050pp::handler::BaseQueryHandler> query_handler_;

  ///\brief the order manager, holding order data of the vda5050::State
  vda5050pp::core::state::OrderManager order_manager_;
  ///\brief the status manager, holding status data of the vda5050::State
  vda5050pp::core::state::StatusManager status_manager_;
  ///\brief the map manager, holding map data of the vda5050::State
  vda5050pp::core::state::MapManager map_manager_;

protected:
  ///
  ///\brief Protected instance constructor (singleton)
  ///
  ///\param config the config to initialize the library with
  ///\param event_manager_options the options for the event managers, if not set, the default
  /// options are used
  ///
  explicit Instance(const vda5050pp::Config &config,
                    vda5050pp::core::events::EventManagerOptions event_manager_options = {});

  ///
  ///\brief Initialize all registered modules and setup spdlog loggers
  ///
  void initializeComponents();

  ///
  ///\brief Deinitialize all registered modules
  ///
  void deInitializeComponents();

public:
  virtual ~Instance() = default;

  ///
  ///\brief Initialize the instance with a config
  ///
  ///\param config the config
  ///\param event_manager_options the options for the event managers, if not set, the default
  ///\return std::weak_ptr<Instance> a pointer to the created instance
  ///
  static std::weak_ptr<Instance> init(
      const vda5050pp::Config &config,
      vda5050pp::core::events::EventManagerOptions event_manager_options = {}) noexcept(true);

  ///
  ///\brief Get a pointer to the current instance
  ///
  ///\return std::weak_ptr<Instance> the current instance
  ///
  static std::weak_ptr<Instance> get() noexcept(true);

  ///
  ///\brief get a reference to the current instance
  ///
  ///\throws VDA5050PPNotInitialized if the instance is not initialized
  ///\return Instance& the current instance
  ///
  static Instance &ref() noexcept(false);

  ///
  ///\brief Reset the current instance
  ///
  static void reset() noexcept(true);

  ///
  ///\brief Register a module
  ///
  ///\param key the key for the module
  ///\param module_ptr  a pointer to the module
  ///\throws VDA5050PPInvalidArgument when a module with this key is already registered
  ///\throws VDA5050PPNullPointer when module_ptr is a nullptr
  ///
  static void registerModule(std::string_view key,
                             std::shared_ptr<vda5050pp::core::Module> module_ptr) noexcept(false);

  ///
  ///\brief Unregister a module by it's key
  ///
  ///\param key the module's key to unregister
  ///\throws VDA5050PPInvalidArgument when a module with this key does not exist
  ///
  static void unregisterModule(std::string_view key) noexcept(false);

  ///
  ///\brief Get a module by it's key
  ///
  ///\param key the key to lookup
  ///\throws VDA5050PPInvalidArgument when a module with this key does not exist
  ///\return std::weak_ptr<vda5050pp::core::Module> the module
  ///
  static std::weak_ptr<vda5050pp::core::Module> lookupModule(std::string_view key) noexcept(false);

  ///
  ///\brief Generate a SubConfig for a module by it's key
  ///
  ///\param key the key of the module
  ///\throws VDA5050PPInvalidArgument when a module with this key does not exist
  ///\return std::shared_ptr<vda5050pp::config::ModuleSubConfig> the sub-config of the module
  ///
  static std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateConfig(
      std::string_view key) noexcept(false);

  ///
  ///\brief Get a list of all keys of the registered modules
  ///
  ///\return std::list<std::string_view> the list of module keys
  ///
  static std::list<std::string_view> registeredModules();

  ///
  ///\brief Start the operation of the library, i.e. go online and send the first state.
  ///
  void start();

  ///
  ///\brief Stop the operation of the library, i.e. send an offline message and disconnect.
  ///
  void stop();

  ///
  ///\brief Get the current config
  ///
  ///\return const vda5050pp::Config& the current config
  ///
  const vda5050pp::Config &getConfig() const;

  ///
  ///\brief Get a rw reference to the ActionEventManager
  ///
  ///\return ActionEventManager&
  ///
  ActionEventManager &getActionEventManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the ActionStatusManager
  ///
  ///\return ActionStatusManager&
  ///
  ActionStatusManager &getActionStatusManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the NavigationEventManager
  ///
  ///\return NavigationEventManager&
  ///
  NavigationEventManager &getNavigationEventManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the NavigationStatusManager
  ///
  ///\return NavigationStatusManager&
  ///
  NavigationStatusManager &getNavigationStatusManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the StatusEventManager
  ///
  ///\return StatusEventManager&
  ///
  StatusEventManager &getStatusEventManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the QueryEventManager
  ///
  ///\return QueryEventManager&
  ///
  QueryEventManager &getQueryEventManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the AgvOrderEventManager
  ///
  ///\return OrderEventManager&
  ///
  OrderEventManager &getAgvOrderEventManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the ControlEventManager
  ///
  ///\return GenericEventManager<vda5050pp::core::events::ControlEvent>&
  ///
  GenericEventManager<vda5050pp::core::events::ControlEvent> &getControlEventManager() noexcept(
      true);

  ///
  ///\brief Get a rw reference to the FactsheetEventManager
  ///
  ///\return GenericEventManager<vda5050pp::core::events::FactsheetEvent>&
  ///
  GenericEventManager<vda5050pp::core::events::FactsheetEvent> &getFactsheetEventManager() noexcept(
      true);

  ///
  ///\brief Get a rw reference to the MessageEventManager
  ///
  ///\return GenericEventManager<vda5050pp::core::events::MessageEvent>&
  ///
  GenericEventManager<vda5050pp::core::events::MessageEvent> &getMessageEventManager() noexcept(
      true);

  ///
  ///\brief Get a rw reference to the OrderEventManager
  ///
  ///\return GenericEventManager<vda5050pp::core::events::OrderEvent>&
  ///
  GenericEventManager<vda5050pp::core::events::OrderEvent> &getOrderEventManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the ValidationEventManager
  ///
  ///\return GenericEventManager<vda5050pp::core::events::ValidationEvent>&
  ///
  GenericEventManager<vda5050pp::core::events::ValidationEvent>
      &getValidationEventManager() noexcept(true);

  ///
  ///\brief Get a ew reference to the InterpreterEventManager
  ///
  ///\return GenericEventManager<vda5050pp::core::events::InterpreterEvent>&
  ///
  GenericEventManager<vda5050pp::core::events::InterpreterEvent>
      &getInterpreterEventManager() noexcept(true);

  ///
  ///\brief Get a rw reference to the StateEventManager
  ///
  ///\return GenericEventManager<vda5050pp::core::events::StateEvent>&
  ///
  GenericEventManager<vda5050pp::core::events::StateEvent> &getStateEventManager() noexcept(true);

  ///
  ///\brief Add a new ActionHandler
  ///
  ///\param action_handler the action handler to add
  ///
  void addActionHandler(
      std::shared_ptr<vda5050pp::handler::BaseActionHandler> action_handler) noexcept(true);

  ///
  ///\brief Get the set of registered ActionHandlers
  ///
  ///\return const std::set<std::shared_ptr<vda5050pp::handler::BaseActionHandler>>& all registered
  /// action handlers
  ///
  const std::set<std::shared_ptr<vda5050pp::handler::BaseActionHandler>> &getActionHandler() const
      noexcept(true);

  ///
  ///\brief Set the current navigation handler
  ///
  ///\param navigation_handler the navigation handler to set
  ///
  void setNavigationHandler(
      std::shared_ptr<vda5050pp::handler::BaseNavigationHandler> navigation_handler) noexcept(true);

  ///
  ///\brief Get the current navigation handler
  ///
  ///\return std::weak_ptr<vda5050pp::handler::BaseNavigationHandler>
  ///
  std::weak_ptr<vda5050pp::handler::BaseNavigationHandler> getNavigationHandler() const
      noexcept(true);

  ///
  ///\brief Set the current QueryHandler
  ///
  ///\param query_handler the query handler to set
  ///
  void setQueryHandler(
      std::shared_ptr<vda5050pp::handler::BaseQueryHandler> query_handler) noexcept(true);

  ///
  ///\brief Get the current query handler
  ///
  ///\return std::weak_ptr<vda5050pp::handler::BaseQueryHandler>
  ///
  std::weak_ptr<vda5050pp::handler::BaseQueryHandler> getQueryHandler() const noexcept(true);

  ///
  ///\brief Get a rw reference to the order manager
  ///
  ///\return vda5050pp::core::state::OrderManager&
  ///
  vda5050pp::core::state::OrderManager &getOrderManager();

  ///
  ///\brief Get a rw reference to the status manager
  ///
  ///\return vda5050pp::core::state::StatusManager&
  ///
  vda5050pp::core::state::StatusManager &getStatusManager();

  ///
  ///\brief Get a rw reference to the map manager
  ///
  ///\return vda5050pp::core::state::MapManager&
  ///
  vda5050pp::core::state::MapManager &getMapManager();
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_INSTANCE_H_
