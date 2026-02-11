// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_INSTANTACTIONS_H_
#define INCLUDE_VDA5050_INSTANTACTIONS_H_

#include <nlohmann/json.hpp>
#include <vector>

#include "Header_vda5050.h"
#include "vda5050/Action.h"

namespace vda5050 {
/// A message containing InstantActions
struct InstantActions {
  /// Message header
  HeaderVDA5050 header;

  /// List of actions to execute
  std::vector<Action> actions;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const InstantActions &other) const {
    if (this->header != other.header) return false;
    if (this->actions != other.actions) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const InstantActions &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const InstantActions &d);
void from_json(const json &j, InstantActions &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_INSTANTACTIONS_H_
