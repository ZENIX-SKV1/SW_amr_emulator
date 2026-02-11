// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ERRORREFERENCE_H_
#define INCLUDE_VDA5050_ERRORREFERENCE_H_

#include <string>
#include <nlohmann/json.hpp>

namespace vda5050 {
/// Desribes a variable and it's value related to the error
struct ErrorReference {
  /// References the type of reference (e. g. headerId, orderId, actionId, …).
  std::string referenceKey;

  /// References the value of the reference key.
  std::string referenceValue;

  bool operator==(const ErrorReference &other) const {
    if (referenceKey != other.referenceKey) return false;
    if (referenceValue != other.referenceValue) return false;
    return true;
  }
  inline bool operator!=(const ErrorReference &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const ErrorReference &d);
void from_json(const json &j, ErrorReference &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ERRORREFERENCE_H_
