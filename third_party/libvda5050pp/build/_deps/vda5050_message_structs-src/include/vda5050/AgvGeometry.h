// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_AGVGEOMETRY_H_
#define VDA5050_AGVGEOMETRY_H_

#include <nlohmann/json.hpp>
#include <vector>

#include "Envelope2d.h"
#include "Envelope3d.h"
#include "vda5050/WheelDefinition.h"

namespace vda5050 {
struct AgvGeometry {
  /// List of wheels, containing wheel-arrangement and geometry.
  std::optional<std::vector<WheelDefinition>> wheelDefinitions;

  /// List of AGV-envelope curves in 2D (german: „Hüllkurven“),
  /// e.g., the mechanical envelopes for unloaded and loaded state,
  /// the safety fields for different speed cases.
  std::optional<std::vector<Envelope2d>> envelopes2d;

  /// List of AGV-envelope curves in 3D (german: „Hüllkurven“).
  std::optional<std::vector<Envelope3d>> envelopes3d;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const AgvGeometry &other) const {
    if (this->wheelDefinitions != other.wheelDefinitions) return false;
    if (this->envelopes2d != other.envelopes2d) return false;
    if (this->envelopes3d != other.envelopes3d) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const AgvGeometry &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const AgvGeometry &d);
void from_json(const json &j, AgvGeometry &d);

}  // namespace vda5050
#endif  // VDA5050_AGVGEOMETRY_H_
