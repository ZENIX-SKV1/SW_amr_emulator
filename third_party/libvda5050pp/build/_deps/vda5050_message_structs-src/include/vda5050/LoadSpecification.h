// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_LOADSPECIFICATION_H_
#define VDA5050_LOADSPECIFICATION_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "vda5050/LoadSet.h"

namespace vda5050 {

struct LoadSpecification {
  /// List of load positions / load handling devices.
  /// This lists contains the valid values for the oarameter “state.loads[].loadPosition”
  /// and for the action parameter “lhd” of the actions pick and drop.
  /// If this list doesn’t exist or is empty, the AGV has no load handling device.
  std::optional<std::vector<std::string>> loadPositions;

  /// list of load-sets that can be handled by the AGV
  std::optional<std::vector<LoadSet>> loadSets;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const LoadSpecification &other) const {
    if (this->loadPositions != other.loadPositions) return false;
    if (this->loadSets != other.loadSets) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const LoadSpecification &other) const {
    return !this->operator==(other);
  }
};

using json = nlohmann::json;
void to_json(json &j, const LoadSpecification &d);
void from_json(const json &j, LoadSpecification &d);

}  // namespace vda5050
#endif  // VDA5050_LOADSPECIFICATION_H_
