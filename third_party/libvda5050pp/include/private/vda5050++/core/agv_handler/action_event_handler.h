//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_AGV_HANDLER_ACTION_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_AGV_HANDLER_ACTION_EVENT_HANDLER_H_

#include <map>
#include <memory>
#include <optional>

#include "vda5050++/core/action_event_manager.h"
#include "vda5050++/core/agv_handler/action_state.h"
#include "vda5050++/core/module.h"
#include "vda5050++/handler/action_state.h"
#include "vda5050++/handler/base_action_handler.h"

namespace vda5050pp::core::agv_handler {

///
///\brief The ActionEventHandler module is responsible for handling action events of
/// the AGV. It forwards those events to the registered user ActionHandlers
///
class ActionEventHandler : public Module {
private:
  ///\brief The event subscriber instance managed by initialize/deinitialize
  std::optional<vda5050pp::core::ScopedActionEventSubscriber> subscriber_;

  ///
  ///\brief The ActionStore stores information of an action that is being handled by the
  /// ActionEventHandler
  ///
  struct ActionStore {
    ///\brief the handled action instance
    std::shared_ptr<const vda5050::Action> action;
    ///\brief the associated state-machine-like ActionState object
    std::shared_ptr<vda5050pp::core::agv_handler::ActionState> action_state;
    ///\brief the associated BaseActionHandler instance
    std::shared_ptr<vda5050pp::handler::BaseActionHandler> action_handler;
    ///\brief the associated user-defined callbacks for the action
    std::shared_ptr<vda5050pp::handler::ActionCallbacks> action_callbacks;
    ///\brief the associated unpacked action parameters
    std::shared_ptr<std::map<std::string, vda5050pp::handler::ParameterValue, std::less<>>>
        action_parameters;
  };

  ///\brief The map of handled actions and their stores
  std::map<std::string, ActionStore, std::less<>> handled_actions_;

  ///
  ///\brief Handles an ActionList event.
  ///
  /// Fills the event return data with information about all handled actions.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///
  void handleActionListEvent(std::shared_ptr<vda5050pp::events::ActionList> data) const
      noexcept(false);

  ///
  ///\brief Handles an ActionValidate event
  ///
  /// Runs the validate member function of the associated BaseActionHandler instance and
  /// fills the event return data with the result.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPNullPointer when the action state is unset
  ///\throws any exception thrown by the associated BaseActionHandler.validate member function
  ///
  void handleValidateEvent(std::shared_ptr<vda5050pp::events::ActionValidate> data) noexcept(false);

  ///
  ///\brief Handles an ActionPrepare event
  ///
  /// Calls the prepare member function of the associated BaseActionHandler instance
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPNullPointer when the action state is unset
  ///\throws any exception thrown by the associated BaseActionHandler.prepare member function
  ///
  void handlePrepareEvent(std::shared_ptr<vda5050pp::events::ActionPrepare> data) noexcept(false);

  ///
  ///\brief Handles an ActionStart event
  ///
  /// Calls the cancel callback of the associated action in the store
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPCallbackNotSet when the callback is not set
  ///\throws VDA5050PPNullPointer when the action state is unset
  ///\throws any exception thrown by the callback
  ///
  void handleCancelEvent(std::shared_ptr<vda5050pp::events::ActionCancel> data) noexcept(false);

  ///
  ///\brief Handles an ActionPause event
  ///
  /// Calls the pause callback of the associated action in the store
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPCallbackNotSet when the callback is not set
  ///\throws VDA5050PPNullPointer when the action state is unset
  ///\throws any exception thrown by the callback
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPCallbackNotSet when the callback is not set
  ///\throws VDA5050PPNullPointer when the action state is unset
  ///\throws any exception thrown by the callback
  ///
  void handlePauseEvent(std::shared_ptr<vda5050pp::events::ActionPause> data) noexcept(false);

  ///
  ///\brief Handles an ActionResume event
  ///
  /// Calls the resume callback of the associated action in the store
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPCallbackNotSet when the callback is not set
  ///\throws VDA5050PPNullPointer when the action state is unset
  ///\throws any exception thrown by the callback
  ///
  void handleResumeEvent(std::shared_ptr<vda5050pp::events::ActionResume> data) noexcept(false);

  ///
  ///\brief Handles an ActionStart event
  ///
  /// Calls the start callback of the associated action in the store
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws VDA5050PPCallbackNotSet when the callback is not set
  ///\throws VDA5050PPNullPointer when the action state is unset
  ///\throws any exception thrown by the callback
  ///
  void handleStartEvent(std::shared_ptr<vda5050pp::events::ActionStart> data) noexcept(false);

  ///
  ///\brief Handles an ActionForget event
  ///
  /// Removes the associated action from the store
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///
  void handleForgetEvent(std::shared_ptr<vda5050pp::events::ActionForget> data) noexcept(false);

  ///
  ///\brief Handles an ActionReset event
  ///
  /// Calls reset on each action handler and resets all handled actions.
  ///
  ///\param data the event data
  ///
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///\throws any exception thrown by BaseActionHandler::reset
  ///
  void handleResetEvent(std::shared_ptr<vda5050pp::events::ActionReset> data) noexcept(false);

  ///
  ///\brief Try to find a handle action by id
  ///
  ///\param action_id the action id to search for
  ///\return std::optional<std::reference_wrapper<ActionStore>> optional reference to the found
  /// action store
  ///
  std::optional<std::reference_wrapper<ActionStore>> tryFindHandledAction(
      std::string_view action_id);

  ///
  ///\brief Try to remove a handled action by id
  ///
  ///\param action_id the action id to remove
  ///\return bool true if the action was removed, false if it did not exist
  ///
  bool tryRemoveActionStore(std::string_view action_id);

public:
  ///
  ///\brief register all event handlers
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) noexcept(false) override;

  ///
  ///\brief unregister all event handlers and remove handled actions
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) noexcept(false) override;

  ///
  ///\brief Return a module description (possibly deprecated)
  ///
  ///\return std::string_view the module's description
  ///
  std::string_view describe() const override;
};

}  // namespace vda5050pp::core::agv_handler

#endif  // VDA5050_2B_2B_CORE_AGV_HANDLER_ACTION_EVENT_HANDLER_H_
