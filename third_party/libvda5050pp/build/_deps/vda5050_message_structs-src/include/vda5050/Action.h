// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ACTION_H_
#define INCLUDE_VDA5050_ACTION_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "vda5050/ActionParameter.h"
#include "vda5050/BlockingType.h"

namespace vda5050 {
/// Describes an action that the AGV can perform.
struct Action {
  /// Name of action as described in the first column
  /// of “Actions and Parameters”. Identifies the
  /// function of the action.
  std::string actionType;

  /// Unique ID to identify the action and map them to
  /// the actionState in the state.Suggestion : Use UUIDs.
  std::string actionId;

  /// Additional information on the action
  std::optional<std::string> actionDescription;

  /// NONE, SOFT, HARD
  BlockingType blockingType = BlockingType::NONE;

  /// Array of actionParameter objects for the indicated
  /// action e.g.deviceId, loadId, external Triggers.
  /// See “Actions and Parameters”.
  std::optional<std::vector<ActionParameter>> actionParameters;

  ///
  ///\brief Compare to another action
  ///
  /// Ignores actionDescription
  ///\param other the other action to compare to
  ///\return equal?
  ///
  inline bool operator==(const Action &other) const noexcept(true) {
    if (actionType != other.actionType) return false;
    if (actionId != other.actionId) return false;
    if (blockingType != other.blockingType) return false;
    if (actionParameters != other.actionParameters) return false;  // TODO: maybe ignore order
    return true;
  }

  ///
  ///\brief Compare to another action
  ///
  /// Ignores actionDescription
  ///\param other the other action to compare to
  ///\return not equal?
  ///
  inline bool operator!=(const Action &other) const noexcept(true) {
    return !this->operator==(other);
  }
};

using json = nlohmann::json;
void to_json(json &j, const Action &d);
void from_json(const json &j, Action &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ACTION_H_
