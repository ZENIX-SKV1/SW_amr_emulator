// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef INCLUDE_VDA5050_AGVACTION_H_
#define INCLUDE_VDA5050_AGVACTION_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "vda5050/ActionParameterFactsheet.h"
#include "vda5050/ActionScope.h"
#include "vda5050/BlockingType.h"

namespace vda5050 {
struct AgvAction {
  /// Unique actionType corresponding to action.actionType.
  std::string actionType;

  /// Free-form text: description of the action.
  std::optional<std::string> actionDescription;

  /// List of allowed scopes for using this action-type.
  /// INSTANT: usable as instantAction.
  /// NODE: usable on nodes.
  /// EDGE: usable on edges.
  /// For example: [„INSTANT“, „NODE“]
  std::vector<ActionScope> actionScopes;

  /// List of parameters
  /// If not defined, the action has no parameters
  std::optional<std::vector<ActionParameterFactsheet>> actionParameters;

  /// Free-form text: description of the resultDescription.
  std::optional<std::string> resultDescription;

  /// Array of possible blocking types for defined action.
  /// Enum {'NONE', 'SOFT', 'HARD'}
  std::optional<std::vector<BlockingType>> blockingTypes;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const AgvAction &other) const {
    if (this->actionType != other.actionType) return false;
    if (this->actionDescription != other.actionDescription) return false;
    if (this->actionScopes != other.actionScopes) return false;
    if (this->actionParameters != other.actionParameters) return false;
    if (this->resultDescription != other.resultDescription) return false;
    if (this->blockingTypes != other.blockingTypes) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const AgvAction &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const AgvAction &d);
void from_json(const json &j, AgvAction &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_AGVACTION_H_
