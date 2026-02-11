// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_PHYSICALPARAMETERS_H_
#define VDA5050_PHYSICALPARAMETERS_H_

#include <nlohmann/json.hpp>
#include <optional>

namespace vda5050 {

struct PhysicalParameters {
  /// [m/s] Minimal controlled continuous speed of the AGV.
  double speedMin = 0.0;

  /// [m/s] Maximum speed of the AGV.
  double speedMax = 0.0;

  /// [Rad/s] Minimal controlled continuous rotation speed of the AGV.
  std::optional<double> angularSpeedMin;

  /// [Rad/s] Maximum rotation speed of the AGV.
  std::optional<double> angularSpeedMax;

  /// [m/s²] Maximum acceleration with maximum load.
  double accelerationMax = 0.0;

  /// [m/s²] Maximum deceleration with maximum load.
  double decelerationMax = 0.0;

  /// [m] Minimum height of AGV.
  double heightMin = 0.0;

  /// [m] Maximum height of AGV.
  double heightMax = 0.0;

  /// [m] Width of AGV.
  double width = 0.0;

  /// [m] Length of AGV.
  double length = 0.0;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const PhysicalParameters &other) const {
    if (this->accelerationMax != other.accelerationMax) return false;
    if (this->angularSpeedMax != other.angularSpeedMax) return false;
    if (this->angularSpeedMin != other.angularSpeedMin) return false;
    if (this->decelerationMax != other.decelerationMax) return false;
    if (this->heightMax != other.heightMax) return false;
    if (this->heightMin != other.heightMin) return false;
    if (this->length != other.length) return false;
    if (this->speedMax != other.speedMax) return false;
    if (this->speedMin != other.speedMin) return false;
    if (this->width != other.width) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const PhysicalParameters &other) const {
    return !this->operator==(other);
  }
};

using json = nlohmann::json;
void to_json(json &j, const PhysicalParameters &d);
void from_json(const json &j, PhysicalParameters &d);

}  // namespace vda5050
#endif  // VDA5050_PHYSICALPARAMETERS_H_
