// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_BOUNDINGBOXREFERENCE_H_
#define INCLUDE_VDA5050_BOUNDINGBOXREFERENCE_H_

#include <optional>
#include <nlohmann/json.hpp>

namespace vda5050 {
/// Point of reference for the location of the bounding box. The point of reference is always the
/// center of the bounding box’s bottom surface (at height = 0) and is described in coordinates of
/// the AGV’s coordinate system.
struct BoundingBoxReference {
  /// x-coordinate of the point of reference.
  double x = 0.0;

  /// y-coordinate of the point of reference.
  double y = 0.0;

  /// z-coordinate of the point of reference.
  double z = 0.0;

  /// Orientation of the loads bounding box. Important for tugger trains etc.
  std::optional<double> theta;

  ///
  ///\brief Check if two BoundingBoxReferences are equal
  ///
  /// used to interact with stl
  ///\param other the BoundingBoxReference to compare to
  ///\return true equal
  ///\return false not equal
  ///
  inline bool operator==(const BoundingBoxReference &other) const noexcept(true) {
    if (this->x != other.x) return false;
    if (this->y != other.y) return false;
    if (this->z != other.z) return false;
    if (this->theta != other.theta) return false;
    return true;
  }

  ///
  ///\brief Check if two BoundingBoxReferences are not equal
  ///
  /// used to interact with stl
  ///\param other the BoundingBoxReference to compare to
  ///\return true not equal
  ///\return false equal
  ///
  inline bool operator!=(const BoundingBoxReference &other) const noexcept(true) {
    return !(this->operator==(other));
  }
};

using json = nlohmann::json;
void to_json(json &j, const BoundingBoxReference &d);
void from_json(const json &j, BoundingBoxReference &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_BOUNDINGBOXREFERENCE_H_
