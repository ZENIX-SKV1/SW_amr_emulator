// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef INCLUDE_VDA5050_POSITION_H
#define INCLUDE_VDA5050_POSITION_H

#include <optional>
#include <nlohmann/json.hpp>

namespace vda5050 {
struct Position {
  /// [m], x-position in AGV-coordinate. system
  double x = 0.0;

  /// [m], y-position in AGV-coordinate. system
  double y = 0.0;

  /// [rad], orientation of wheel in AGV-coordinate system Necessary for fixed wheels.
  std::optional<double> theta;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Position &other) const {
    if (this->x != other.x) return false;
    if (this->y != other.y) return false;
    if (this->theta != other.theta) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Position &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Position &d);
void from_json(const json &j, Position &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_POSITION_H