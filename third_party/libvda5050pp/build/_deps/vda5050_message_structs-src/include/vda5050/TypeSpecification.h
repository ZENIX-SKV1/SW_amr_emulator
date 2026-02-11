// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_TYPESPECIFICATION_H_
#define VDA5050_TYPESPECIFICATION_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace vda5050 {

struct TypeSpecification {
  /// Free text generalized series name as specified by manufacturer.
  std::string seriesName;

  /// Free text human readable description of the AGV type series.
  std::optional<std::string> seriesDescription;

  /// Simplified description of AGV kinematics-type.
  /// [DIFF, OMNI, THREEWHEEL]
  /// DIFF: differential drive
  /// OMNI: omni-directional vehicle
  /// THREEWHEEL: three-wheel-driven vehicle or vehicle with similar kinematics
  std::string agvKinematic;

  /// Simplified description of AGV class.
  /// [FORKLIFT, CONVEYOR, TUGGER, CARRIER]
  /// FORKLIFT: forklift.
  /// CONVEYOR: AGV with conveyors on it.
  /// TUGGER: tugger.
  /// CARRIER: load carrier with or without lifting unit.
  std::string agvClass;

  /// [kg], Maximum loadable mass.
  double maxLoadMass = 0.0;

  /// Simplified description of localization type.
  /// Example values:
  /// NATURAL: natural landmarks;
  /// REFLECTOR: laser reflectors;
  /// RFID: RFID-tags;
  /// DMC: data matrix code;
  /// SPOT: magnetic spots;
  /// GRID: magnetic grid.
  std::vector<std::string> localizationTypes;

  /// List of path planning types supported by the AGV, sorted by priority.
  /// Example values:
  /// PHYSICAL_LINE_GUIDED: No path planning, AGV follows physical installed paths.
  /// VIRTUAL_LINE_GUIDED: AGV goes fixed (virtual) paths.
  /// AUTONOMOUS: AGV plans its path autonomously.
  std::vector<std::string> navigationTypes;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const TypeSpecification &other) const {
    if (this->seriesName != other.seriesName) return false;
    if (this->seriesDescription != other.seriesDescription) return false;
    if (this->agvKinematic != other.agvKinematic) return false;
    if (this->agvClass != other.agvClass) return false;
    if (this->maxLoadMass != other.maxLoadMass) return false;
    if (this->localizationTypes != other.localizationTypes) return false;
    if (this->navigationTypes != other.navigationTypes) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const TypeSpecification &other) const {
    return !this->operator==(other);
  }
};

using json = nlohmann::json;
void to_json(json &j, const TypeSpecification &d);
void from_json(const json &j, TypeSpecification &d);

}  // namespace vda5050
#endif  // VDA5050_TYPESPECIFICATION_H_
