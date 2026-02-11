// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ACTIONPARAMETER_H_
#define INCLUDE_VDA5050_ACTIONPARAMETER_H_

#include <nlohmann/json.hpp>
#include <string>

namespace vda5050 {

using json = nlohmann::json;

/// A key-value-pair
/// The value will be deserialized to an actual datatype
struct ActionParameter {
  /// Action key
  std::string key;

  /// Value of the action key
  json value;

  ///
  ///\brief Check if equal to another ActionParameter
  ///
  /// Does not check value.value
  ///\param other ActionParamater to compare to
  ///\return equal?
  ///
  inline bool operator==(const ActionParameter &other) const noexcept(true) {
    if (key != other.key) return false;
    if (value != other.value) return false;
    return true;
  }

  ///
  ///\brief Check if not equal to another ActionParameter
  ///
  /// Does not check value.value
  ///\param other ActionParamater to compare to
  ///\return not equal?
  ///
  inline bool operator!=(const ActionParameter &other) const noexcept(true) {
    return !this->operator==(other);
  }
};

void to_json(json &j, const ActionParameter &d);
void from_json(const json &j, ActionParameter &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ACTIONPARAMETER_H_
