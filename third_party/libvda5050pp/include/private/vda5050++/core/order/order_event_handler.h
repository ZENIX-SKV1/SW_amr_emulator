//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_ORDER_ORDER_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_ORDER_ORDER_EVENT_HANDLER_H_

#include "vda5050++/core/action_event_manager.h"
#include "vda5050++/core/events/interpreter_event.h"
#include "vda5050++/core/generic_event_manager.h"
#include "vda5050++/core/module.h"
#include "vda5050++/core/navigation_event_manager.h"
#include "vda5050++/core/order/scheduler.h"

namespace vda5050pp::core::order {

///
///\brief The OrderEventHandler module handles all events related to the order execution. And
/// processes them with the Scheduler
///
class OrderEventHandler : public vda5050pp::core::Module {
private:
  ///\brief The scheduler instance
  std::optional<vda5050pp::core::order::Scheduler> scheduler_;

  ///\brief Interpreter event subscriber
  std::optional<GenericEventManager<vda5050pp::core::events::InterpreterEvent>::ScopedSubscriber>
      interpreter_subscriber_;
  ///\brief Action event subscriber
  std::optional<vda5050pp::core::ScopedActionStatusSubscriber> action_event_subscriber_;
  ///\brief Navigation event subscriber
  std::optional<vda5050pp::core::ScopedNavigationStatusSubscriber> navigation_event_subscriber_;

  ///\brief state event subscriber
  std::optional<GenericEventManager<vda5050pp::core::events::StateEvent>::ScopedSubscriber>
      state_event_subscriber_;

  ///
  ///\brief Handle normal events from the interpreter (non-instant actions)
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleYieldNormal(std::shared_ptr<vda5050pp::core::events::InterpreterEvent> evt);

  ///
  ///\brief Handle instant action events from the interpreter
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleYieldInstantActionGroup(
      std::shared_ptr<vda5050pp::core::events::YieldInstantActionGroup> evt);

  ///
  ///\brief Handle a new action event from the interpreter and forwards it as an ActionPrepare event
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleYieldNewAction(std::shared_ptr<vda5050pp::core::events::YieldNewAction> evt) const
      noexcept(false);

  ///
  ///\brief Handle InterpreterDone event, which will start processing previously received
  /// interpreter events
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleInterpreterDone(std::shared_ptr<vda5050pp::core::events::InterpreterDone> evt);

  ///
  ///\brief Handle a failed action event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleActionFailed(std::shared_ptr<vda5050pp::events::ActionStatusFailed> evt);

  ///
  ///\brief Handle a finished action event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleActionFinished(std::shared_ptr<vda5050pp::events::ActionStatusFinished> evt);

  ///
  ///\brief Handle a initializing action event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleActionInitializing(std::shared_ptr<vda5050pp::events::ActionStatusInitializing> evt);

  ///
  ///\brief Handle a paused action event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleActionPaused(std::shared_ptr<vda5050pp::events::ActionStatusPaused> evt);

  ///
  ///\brief Handle a running action event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleActionRunning(std::shared_ptr<vda5050pp::events::ActionStatusRunning> evt);

  ///
  ///\brief Handle a navigation control event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleNavigationControl(std::shared_ptr<vda5050pp::events::NavigationStatusControl> evt);

  ///
  ///\brief Handle a navigation node reached event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleNavigationNode(std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached> evt);

  ///
  ///\brief Handle a order control reached event and forwards it to the scheduler.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleOrderControl(
      std::shared_ptr<vda5050pp::core::events::InterpreterOrderControl> evt) noexcept(false);

  ///
  ///\brief Handle a operating mode changed event. If it switches to manual, reset the current
  /// order.
  ///
  ///\param evt The event
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///
  void handleOperatingModeChanged(
      std::shared_ptr<vda5050pp::core::events::OperatingModeChangedEvent> evt) noexcept(false);

public:
  ///
  ///\brief Initializes all subscribers and the scheduler
  ///
  ///\param instance The current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribes all subscribers and deinitializes the scheduler
  ///
  ///\param instance The current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a brief description of the module (possibly deprecated)
  ///
  ///\return std::string_view the description
  ///
  std::string_view describe() const override;
};
}  // namespace vda5050pp::core::order

#endif  // VDA5050_2B_2B_CORE_ORDER_ORDER_EVENT_HANDLER_H_
