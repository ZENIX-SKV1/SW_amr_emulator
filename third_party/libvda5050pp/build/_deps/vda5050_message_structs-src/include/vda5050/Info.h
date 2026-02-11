// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_INFO_H_
#define INCLUDE_VDA5050_INFO_H_

#include <optional>
#include <string>
#include <vector>

#include "vda5050/InfoLevel.h"
#include "vda5050/InfoReference.h"
#include <nlohmann/json.hpp>

namespace vda5050 {
/// Message type for debugging and or visualization information
struct Info {
  /// Type/name of info
  std::string infoType;

  /// Array of references to identify the source of
  /// the error. (e. g. headerId,orderId,actionId,...)
  std::optional<std::vector<InfoReference>> infoReferences;

  /// Info description
  std::optional<std::string> infoDescription;

  /// Debugging or visualization
  InfoLevel infoLevel = InfoLevel::DEBUG;

  bool operator==(const Info &other) const {
    if (infoType != other.infoType) return false;
    if (infoReferences != other.infoReferences) return false;
    if (infoDescription != other.infoDescription) return false;
    if (infoLevel != other.infoLevel) return false;
    return true;
  }
  inline bool operator!=(const Info &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Info &d);
void from_json(const json &j, Info &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_INFO_H_
