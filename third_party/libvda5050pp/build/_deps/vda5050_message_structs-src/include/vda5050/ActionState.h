// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ACTIONSTATE_H_
#define INCLUDE_VDA5050_ACTIONSTATE_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/ActionStatus.h"

namespace vda5050 {
/// VD(M)A 5050 ActionState
struct ActionState {
  /// action_ID
  std::string actionId;

  /// actionType of the action.
  /// Optional: Only for informational purposes.
  /// Order knows the type.
  /// Note: Not explcitly flagged optional after VD(M)A 5050,
  /// but more consisten after the description
  std::optional<std::string> actionType;

  /// Additional information on the current action
  std::optional<std::string> actionDescription;

  /// The current progress of the action
  ActionStatus actionStatus = ActionStatus::WAITING;

  /// Description of the result, e.g. the result of a RFID-read.
  /// Errors will be transmitted in errors. Examples for results are
  /// given in 6.5
  std::optional<std::string> resultDescription;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const ActionState &other) const {
    if (this->actionId != other.actionId) return false;
    if (this->actionType != other.actionType) return false;
    if (this->actionDescription != other.actionDescription) return false;
    if (this->actionStatus != other.actionStatus) return false;
    if (this->resultDescription != other.resultDescription) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const ActionState &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const ActionState &d);
void from_json(const json &j, ActionState &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ACTIONSTATE_H_
