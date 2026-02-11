// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
//

#ifndef VDA5050_ACTIONPARAMETERFACTSHEET_H_
#define VDA5050_ACTIONPARAMETERFACTSHEET_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/ValueDataType.h"

namespace vda5050 {
struct ActionParameterFactsheet {
  /// Key-String for Parameter.
  std::string key;

  /// Data type of Value, possible data types are: BOOL, NUMBER, INTEGER, FLOAT, STRING, OBJECT,
  /// ARRAY.
  ValueDataType valueDataType = ValueDataType::OBJECT;

  /// Free-form text: description of the parameter.
  std::optional<std::string> description;

  /// "true": optional parameter.
  std::optional<bool> isOptional;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const ActionParameterFactsheet &other) const {
    if (this->key != other.key) return false;
    if (this->valueDataType != other.valueDataType) return false;
    if (this->description != other.description) return false;
    if (this->isOptional != other.isOptional) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const ActionParameterFactsheet &other) const {
    return !this->operator==(other);
  }
};

using json = nlohmann::json;
void to_json(json &j, const ActionParameterFactsheet &d);
void from_json(const json &j, ActionParameterFactsheet &d);

}  // namespace vda5050
#endif  // VDA5050_ACTIONPARAMETERFACTSHEET_H_
