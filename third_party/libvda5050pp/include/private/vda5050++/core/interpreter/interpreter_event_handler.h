//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_INTERPRETER_INTERPRETER_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_INTERPRETER_INTERPRETER_EVENT_HANDLER_H_

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "vda5050++/core/events/event_control_blocks.h"
#include "vda5050++/core/module.h"

namespace vda5050pp::core::interpreter {

///
///\brief The InterpreterEventHandler module handles ValidInstantActionMessageEvents and
/// ValidOrderMessageEvents
/// and then interprets them into InterpreterEvents, which are atomic actions for the AGV.
/// The Scheduler then schedules those atomic actions.
///
/// If the ValidInstantActionMessageEvent is a control instant action, the InterpreterEventHandler
/// will Create a control block to handle the flow of the action. And eventually control the
/// interpreter.
///
/// Because Actions are handled the ActionValidate and FactsheetControlActionList events must
/// be handled by the InterpreterEventHandler.
///
class InterpreterEventHandler : public vda5050pp::core::Module {
private:
  ///\brief The subscriber for the message events
  std::optional<
      vda5050pp::core::GenericEventManager<vda5050pp::core::events::MessageEvent>::ScopedSubscriber>
      message_subscriber_;
  ///\brief The subscriber for the action events
  std::optional<vda5050pp::core::ScopedActionEventSubscriber> action_subscriber_;
  ///\brief The subscriber for the action factsheet events
  std::optional<vda5050pp::core::GenericEventManager<
      vda5050pp::core::events::FactsheetEvent>::ScopedSubscriber>
      factsheet_subscriber_;

  ///\brief Storage for all active control blocks
  std::map<std::string, std::shared_ptr<vda5050pp::core::events::EventControlBlock>, std::less<>>
      active_control_blocks_;

  ///
  ///\brief Handle a ValidInstantActionMessageEvent.
  ///
  /// If this is a control action, create a control block and handle the flow of the action.
  /// Otherwise forward the action-groups to the scheduler
  ///
  ///\param data the event data
  ///\throws vda5050pp::VDA5050PPInvalidEventData if the data is empty
  ///
  void handleValidInstantActions(
      std::shared_ptr<vda5050pp::core::events::ValidInstantActionMessageEvent>
          data) noexcept(false);

  ///
  ///\brief Handle a ValidOrderMessageEvent.
  ///
  /// This iterates over the whole order and yields the following events in order they shall be
  /// executed:
  /// - YieldNewAction for each encountered action
  /// - YieldActionGroup for each encountered action group
  /// - YieldNavigationStep for each encountered navigation step
  /// - YieldGraphReplacement or YieldGraphExtension in the end
  /// - InterpreterDone as the last event for this order
  ///
  ///\param data the event data
  ///\throws vda5050pp::VDA5050PPInvalidEventData if the data is empty
  ///
  void handleValidOrder(std::shared_ptr<vda5050pp::core::events::ValidOrderMessageEvent> data) const
      noexcept(false);

  ///
  ///\brief Handle an ActionValidateEvent.
  ///
  /// This only fills in the validation result for control actions.
  ///
  ///\param data the event data
  ///\throws vda5050pp::VDA5050PPInvalidEventData if the data is empty
  ///
  void handleActionValidateEvent(std::shared_ptr<vda5050pp::events::ActionValidate> data) const
      noexcept(false);

  ///
  ///\brief Fills in a list of all control actions.
  ///
  ///\param data the event data
  ///\throws vda5050pp::VDA5050PPInvalidEventData if the data is empty
  ///\throws vda5050pp::VDA5050PPSynchronizedEventNotAcquired if the result token could not be
  /// acquired
  ///
  void handleFactsheetControlActionListEvent(
      std::shared_ptr<vda5050pp::core::events::FactsheetControlActionListEvent> data) const
      noexcept(false);

public:
  ///
  ///\brief Setup all subscribers for the event handler.
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsuscribe all subscribers.
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a description of the module. (possibly deprecated)
  ///
  ///\return std::string_view the description
  ///
  std::string_view describe() const override;
};

}  // namespace vda5050pp::core::interpreter

#endif  // VDA5050_2B_2B_CORE_INTERPRETER_INTERPRETER_EVENT_HANDLER_H_
