// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef INCLUDE_VDA5050_CORRIDOR_H_
#define INCLUDE_VDA5050_CORRIDOR_H_

#include <nlohmann/json.hpp>
#include <optional>

#include "vda5050/CorridorRefPoint.h"

namespace vda5050 {
struct Corridor {
  /// [m] Range: [0.0 ... float64.max]
  /// Defines the width of the corridor in meters to the left related to
  /// the trajectory of the vehicle (see Figure 13).
  double leftWidth = 0.0;

  /// [m] Range: [0.0 ... float64.max]
  /// Defines the width of the corridor in meters to the right related to
  /// the trajectory of the vehicle (see Figure 13).
  double rightWidth = 0.0;

  /// Defines whether the boundaries are valid for the kinematic center
  /// or the contour of the vehicle. If not specified the boundaries are
  /// valid to the vehicles kinematic center.
  std::optional<CorridorRefPoint> corridorRefPoint;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Corridor &other) const {
    if (this->leftWidth != other.leftWidth) return false;
    if (this->rightWidth != other.rightWidth) return false;
    if (this->corridorRefPoint != other.corridorRefPoint) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Corridor &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Corridor &d);
void from_json(const json &j, Corridor &d);

}  // namespace vda5050

#endif  // INCLUDE_VDA5050_CORRIDOR_H_
