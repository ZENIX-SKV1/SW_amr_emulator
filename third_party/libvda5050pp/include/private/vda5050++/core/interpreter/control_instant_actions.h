// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_INTERPRETER_CONTROL_INSTANT_ACTIONS_H_
#define VDA5050_2B_2B_CORE_INTERPRETER_CONTROL_INSTANT_ACTIONS_H_

#include <vda5050/Action.h>
#include <vda5050/AgvAction.h>
#include <vda5050/Error.h>

#include <list>

#include "vda5050++/core/events/event_control_blocks.h"
#include "vda5050++/misc/action_context.h"

namespace vda5050pp::core::interpreter {

///
///\brief Check if the action is a control instant action, i.e. startPause, stopPause, cancelOrder,
/// facsheetRequest or stateRequest
///
///\param instant_action the instant action to check
///\return true if the instant action is a control instant action, false otherwise
///
bool isControlInstantAction(const vda5050::Action &instant_action);

///
///\brief Validate a control instant action
///
///\param instant_action the instant action to validate
///\param ctxt the context of the action
///\return std::list<vda5050::Error> a list of errors, empty if the action is valid
///\throws VDA5050PPInvalidArgument if the action is not a control instant action
///
std::list<vda5050::Error> validateControlInstantAction(const vda5050::Action &instant_action,
                                                       vda5050pp::misc::ActionContext ctxt);

///
///\brief Return a list of a all control actions (for the factsheet)
///
///\return std::shared_ptr<std::list<vda5050::AgvAction>> the list of control actions
///
std::shared_ptr<std::list<vda5050::AgvAction>> listControlActions() noexcept(false);

///
///\brief Create a control block to handle the flow of a cancel action
///
/// Schema:
/// [start-cancel] -> [wait for idle] -> [set action FINISHED]
///
///\param action the cancelOrder action
///\return std::shared_ptr<vda5050pp::core::events::EventControlBlock> the control block
///
std::shared_ptr<vda5050pp::core::events::EventControlBlock> makeCancelControlBlock(
    std::shared_ptr<const vda5050::Action> action);

///
///\brief Create a control block to handle the flow of a startPause action
///
/// Schema:
/// [start-pause] |-> [wait for pausing] -> [wait for pause] |-> [set action FINISHED]
///               |-> [wait for pause_idle]                  |
///
///\param action the startPause action
///\return std::shared_ptr<vda5050pp::core::events::EventControlBlock>  the control block
///
std::shared_ptr<vda5050pp::core::events::EventControlBlock> makePauseControlBlock(
    std::shared_ptr<const vda5050::Action> action);

///
///\brief Create a control block to handle the flow of a stopPause action
///
/// Schema:
/// [start-resume] |-> [wait for resuming] -> [wait for running] |-> [set action FINISHED]
///                |-> [wait for idle]                           |
///
///\param action the stopPause action
///\return std::shared_ptr<vda5050pp::core::events::EventControlBlock> the control block
///
std::shared_ptr<vda5050pp::core::events::EventControlBlock> makeResumeControlBlock(
    std::shared_ptr<const vda5050::Action> action);

///
///\brief Create a control block to handle factsheetRequest actions
///
/// Schema:
/// [send factsheet msg] -> [set action to FINISHED]
///
///\param action the factsheetRequest action
///\return std::shared_ptr<vda5050pp::core::events::EventControlBlock> the control block
///
std::shared_ptr<vda5050pp::core::events::EventControlBlock> makeFactsheetRequestControlBlock(
    std::shared_ptr<const vda5050::Action> action);

///
///\brief Create a control block to handle stateRequest actions
///
/// Schema:
/// [set action to FINISHED]
/// This automatically triggers a state update, since the action status changed.
///
///\param action the stateRequest action
///\return std::shared_ptr<vda5050pp::core::events::EventControlBlock> the control block
///
std::shared_ptr<vda5050pp::core::events::EventControlBlock> makeStateRequestControlBlock(
    std::shared_ptr<const vda5050::Action> action);

///
///\brief Make a control block for an instant action
///
/// Depending on the type of the action the correct control block is returned.
/// It must be enabled to start the control instant action.
///
///\param action the instant action
///\return std::shared_ptr<vda5050pp::core::events::EventControlBlock> the control block
///\throws VDA5050PPInvalidArgument if the action is not a control instant action
///\throws VDA5050PPNullPointer if the action is nullptr
///
std::shared_ptr<vda5050pp::core::events::EventControlBlock> makeControlInstantActionControlBlock(
    std::shared_ptr<const vda5050::Action> action);

}  // namespace vda5050pp::core::interpreter

#endif  // VDA5050_2B_2B_CORE_INTERPRETER_CONTROL_INSTANT_ACTIONS_H_
