// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_VELOCITY_H_
#define INCLUDE_VDA5050_VELOCITY_H_

#include <optional>
#include <nlohmann/json.hpp>

namespace vda5050 {
/// Velocity
/// each member is optional
struct Velocity {
  /// The AGVs velocity in its x direction
  std::optional<double> vx;

  /// The AGVs velocity in its y direction
  std::optional<double> vy;

  /// The AGVs turning speed around its z axis
  std::optional<double> omega;

  inline bool operator==(const Velocity &other) const {
    if (vx != other.vx) return false;
    if (vy != other.vy) return false;
    if (omega != other.omega) return false;
    return true;
  }
  inline bool operator!=(const Velocity &other) const { return !(this->operator==(other)); }
};

using json = nlohmann::json;
void to_json(json &j, const Velocity &d);
void from_json(const json &j, Velocity &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_VELOCITY_H_
