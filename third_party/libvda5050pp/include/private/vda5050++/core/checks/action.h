//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_CHECKS_ACTION_H_
#define VDA5050_2B_2B_CORE_CHECKS_ACTION_H_

#include <vda5050/Action.h>
#include <vda5050/Error.h>

#include <list>

#include "vda5050++/agv_description/action_declaration.h"
#include "vda5050++/handler/base_action_handler.h"

namespace vda5050pp::core::checks {

///
///\brief Check if an action matches the type of a given action declaration.
///
///\param action_declaration the action declaration to check against.
///\param action the action to check.
///\return true if the action matches the type of the action declaration, false otherwise.
///
bool matchActionType(const vda5050pp::agv_description::ActionDeclaration &action_declaration,
                     const vda5050::Action &action) noexcept(true);

///
///\brief Validate an action with a declaration in a given context and fill the parameters map.
///
///\param action the action to validate.
///\param context the context in which the action is validated.
///\param action_declaration the action declaration to check against.
///\param parameters the parameters map to fill with values from the action.
///\return std::list<vda5050::Error> A list of validation errors, if empty the action is valid.
///
std::list<vda5050::Error> validateActionWithDeclaration(
    const vda5050::Action &action, vda5050pp::misc::ActionContext context,
    const vda5050pp::agv_description::ActionDeclaration &action_declaration,
    std::optional<std::reference_wrapper<vda5050pp::handler::ParametersMap>> parameters =
        std::nullopt) noexcept(false);

///
///\brief Check if an action can be executed in a given context.
///
///\param action the action to check.
///\param context the context in which the action is validated.
///\param action_declaration the action declaration to check against.
///\return std::list<vda5050::Error> A list of validation errors, if empty the action is valid.
///
std::list<vda5050::Error> contextCheck(
    const vda5050::Action &action, vda5050pp::misc::ActionContext context,
    const vda5050pp::agv_description::ActionDeclaration &action_declaration);

///
///\brief Check if an action has a unique ID. Checks against the current state and a seen set.
///
///\param action the action to check.
///\param seen the seen set. It will be extended with the id of the given action.
///\param context The context the action appears in. Used to set proper error types.
///\return std::list<vda5050::Error> the error list
///
std::list<vda5050::Error> uniqueActionId(const vda5050::Action &action,
                                         std::set<std::string_view, std::less<>> &seen,
                                         vda5050pp::misc::ActionContext context) noexcept(false);

///
///\brief Check if an control instant action can be executed in the current order state.
///
///\param action the control instant action to check.
///\return std::list<vda5050::Error> A list of validation errors, if empty the action is valid.
///
std::list<vda5050::Error> controlActionFeasible(const vda5050::Action &action) noexcept(false);

}  // namespace vda5050pp::core::checks

#endif  // VDA5050_2B_2B_CORE_CHECKS_ACTION_H_
