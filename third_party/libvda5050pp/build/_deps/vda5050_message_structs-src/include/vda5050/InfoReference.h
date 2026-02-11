// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_INFOREFERENCE_H_
#define INCLUDE_VDA5050_INFOREFERENCE_H_

#include <string>
#include <nlohmann/json.hpp>

namespace vda5050 {
struct InfoReference {
  /// References the type of reference (e.g. orderId, headerId, actionId, ...)
  std::string referenceKey;

  /// References the value, which belongs to the key.
  std::string referenceValue;

  bool operator==(const InfoReference &other) const {
    if (referenceKey != other.referenceKey) return false;
    if (referenceValue != other.referenceValue) return false;
    return true;
  }
  inline bool operator!=(const InfoReference &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const InfoReference &d);
void from_json(const json &j, InfoReference &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_INFOREFERENCE_H_
