// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_POLYGONPOINT_H_
#define VDA5050_POLYGONPOINT_H_

#include <nlohmann/json.hpp>

namespace vda5050 {
struct PolygonPoint {
  /// [m], x-position of polygon-point.
  double x = 0.0;

  /// [m], y-position of polygon-point.
  double y = 0.0;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const PolygonPoint &other) const {
    if (this->x != other.x) return false;
    if (this->y != other.y) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const PolygonPoint &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const PolygonPoint &d);
void from_json(const json &j, PolygonPoint &d);

}  // namespace vda5050
#endif  // VDA5050_POLYGONPOINT_H_
