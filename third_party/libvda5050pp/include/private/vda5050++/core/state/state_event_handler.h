//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_STATE_STATE_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_STATE_STATE_EVENT_HANDLER_H_

#include <optional>

#include "vda5050++/core/events/interpreter_event.h"
#include "vda5050++/core/events/message_event.h"
#include "vda5050++/core/module.h"

namespace vda5050pp::core::state {

///
///\brief The StateEventHandler module handles all events related to the state of the AGV
/// and stores them in the {Order,Status}Manager
///
class StateEventHandler : public vda5050pp::core::Module {
private:
  std::optional<
      vda5050pp::core::GenericEventManager<vda5050pp::core::events::MessageEvent>::ScopedSubscriber>
      messages_subscriber_;
  std::optional<vda5050pp::core::GenericEventManager<
      vda5050pp::core::events::InterpreterEvent>::ScopedSubscriber>
      interpreter_subscriber_;

  std::optional<
      vda5050pp::core::GenericEventManager<vda5050pp::core::events::OrderEvent>::ScopedSubscriber>
      order_subscriber_;

  std::optional<vda5050pp::core::ScopedNavigationStatusSubscriber> navigation_subscriber_;

  std::optional<vda5050pp::core::ScopedStatusEventSubscriber> status_subscriber_;

  ///
  ///\brief Handle amessage error event and forward it to the state.
  ///
  ///\param std::shared_ptr<vda5050pp::core::events::MessageErrorEvent> data the error to forward
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleMessageErrorEvent(
      std::shared_ptr<vda5050pp::core::events::MessageErrorEvent> data) const noexcept(false);

  ///
  ///\brief Handle a graph extension event and forward it to the state. A
  /// NavigationBaseIncreased
  /// event is also dispatched.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleGraphExtensionEvent(
      std::shared_ptr<vda5050pp::core::events::YieldGraphExtension> data) const noexcept(false);

  ///
  ///\brief Handle a graph replacement event and forward it to the state. A NavigationBaseIncreased
  /// event is also dispatched.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleGraphReplacementEvent(
      std::shared_ptr<vda5050pp::core::events::YieldGraphReplacement> data) const noexcept(false);

  ///
  ///\brief Handle a new action event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleNewActionEvent(std::shared_ptr<vda5050pp::core::events::YieldNewAction> data) const
      noexcept(false);

  ///
  ///\brief Handle a clear actions event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleClearActions(std::shared_ptr<vda5050pp::core::events::YieldClearActions> data) const
      noexcept(false);

  ///
  ///\brief Handle a clear actions event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOrderNewLastNodeId(
      std::shared_ptr<vda5050pp::core::events::OrderNewLastNodeId> data) const noexcept(false);

  ///
  ///\brief Handle a action status changed event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOrderActionStatusChanged(
      std::shared_ptr<vda5050pp::core::events::OrderActionStatusChanged> data) const
      noexcept(false);

  ///
  ///\brief Handle a order status event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOrderStatus(std::shared_ptr<vda5050pp::core::events::OrderStatus> data) const
      noexcept(false);

  ///
  ///\brief Handle a order clear after cancel event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOrderClearAfterCancel(
      std::shared_ptr<vda5050pp::core::events::OrderClearAfterCancel> data) const;

  ///\brief Handle a order clear after a reset and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOrderClearAfterReset(
      std::shared_ptr<vda5050pp::core::events::OrderClearAfterReset> data) const;

  ///
  ///\brief Handle a navigation status position event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleNavigationStatusPosition(
      std::shared_ptr<vda5050pp::events::NavigationStatusPosition> data) const noexcept(false);

  ///
  ///\brief Handle a navigation status velocity event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleNavigationStatusVelocity(
      std::shared_ptr<vda5050pp::events::NavigationStatusVelocity> data) const noexcept(false);

  ///
  ///\brief Handle a navigation status distance since last node event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleNavigationStatusDistanceSinceLastNode(
      std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode> data) const
      noexcept(false);

  ///
  ///\brief Handle a navigation status driving event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleNavigationStatusDriving(
      std::shared_ptr<vda5050pp::events::NavigationStatusDriving> data) const noexcept(false);

  ///
  ///\brief Handle a navigation status node reached event and forward it to the state.
  /// Forward it to the OrderManager if a new node was reached.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleNavigationStatusNodeReached(
      std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached> data) const noexcept(false);

  ///
  ///\brief Handle a load add event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleLoadAdd(std::shared_ptr<vda5050pp::events::LoadAdd> data) const noexcept(false);

  ///
  ///\brief Handle a load remove event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleLoadRemove(std::shared_ptr<vda5050pp::events::LoadRemove> data) const noexcept(false);

  ///
  ///\brief Handle a load get event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleLoadsGet(std::shared_ptr<vda5050pp::events::LoadsGet> data) const noexcept(false);

  ///
  ///\brief Handle a load alter event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleLoadsAlter(std::shared_ptr<vda5050pp::events::LoadsAlter> data) const noexcept(false);

  ///
  ///\brief Handle a operating mode set event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOperatingModeSet(std::shared_ptr<vda5050pp::events::OperatingModeSet> data) const
      noexcept(false);

  ///
  ///\brief Handle a operating mode get event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOperatingModeGet(std::shared_ptr<vda5050pp::events::OperatingModeGet> data) const
      noexcept(false);

  ///
  ///\brief Handle a operating mode alter event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleOperatingModeAlter(std::shared_ptr<vda5050pp::events::OperatingModeAlter> data) const
      noexcept(false);

  ///
  ///\brief Handle a battery state set event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleBatteryStateSet(std::shared_ptr<vda5050pp::events::BatteryStateSet> data) const
      noexcept(false);

  ///
  ///\brief Handle a battery state get event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleBatteryStateGet(std::shared_ptr<vda5050pp::events::BatteryStateGet> data) const
      noexcept(false);

  ///
  ///\brief Handle a battery state alter event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleBatteryStateAlter(std::shared_ptr<vda5050pp::events::BatteryStateAlter> data) const
      noexcept(false);

  ///
  ///\brief Handle a safety state set event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleSafetyStateSet(std::shared_ptr<vda5050pp::events::SafetyStateSet> data) const
      noexcept(false);

  ///
  ///\brief Handle a safety state get event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleSafetyStateGet(std::shared_ptr<vda5050pp::events::SafetyStateGet> data) const
      noexcept(false);

  ///
  ///\brief Handle a safety state alter event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleSafetyStateAlter(std::shared_ptr<vda5050pp::events::SafetyStateAlter> data) const
      noexcept(false);

  ///
  ///\brief Handle a request new base event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleRequestNewBase(std::shared_ptr<vda5050pp::events::RequestNewBase> data) const
      noexcept(false);

  ///
  ///\brief Handle a errors add event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleErrorAdd(std::shared_ptr<vda5050pp::events::ErrorAdd> data) const noexcept(false);

  ///
  ///\brief Handle a errors alter event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleErrorsAlter(std::shared_ptr<vda5050pp::events::ErrorsAlter> data) const
      noexcept(false);

  ///
  ///\brief Handle a infos add event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleInfoAdd(std::shared_ptr<vda5050pp::events::InfoAdd> data) const noexcept(false);

  ///
  ///\brief Handle a infos alter event and forward it to the state.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleInfosAlter(std::shared_ptr<vda5050pp::events::InfosAlter> data) const noexcept(false);

  ///
  ///\brief Handle an AddMap event
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleAddMap(std::shared_ptr<vda5050pp::events::AddMap> data) const noexcept(false);

  ///
  ///\brief Handle an EnableMap event
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleEnableMap(std::shared_ptr<vda5050pp::events::EnableMap> data) const noexcept(false);

  ///
  ///\brief Handle an DeleteMap event
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPNullPointer if data is nullptr
  ///
  void handleDeleteMap(std::shared_ptr<vda5050pp::events::DeleteMap> data) const noexcept(false);

public:
  ///
  ///\brief Setup all subscribers.
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribe all subscribers.
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a brief description of the module (possibly deprecated)
  ///
  ///\return std::string_view the module's description
  ///
  std::string_view describe() const override;

  ///
  ///\brief Generate a new SubConfig for this module
  ///
  ///\return std::shared_ptr<vda5050pp::config::ModuleSubConfig> the module's sub-config
  ///
  std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateSubConfig() const override;
};

}  // namespace vda5050pp::core::state

#endif  // VDA5050_2B_2B_CORE_STATE_STATE_EVENT_HANDLER_H_
