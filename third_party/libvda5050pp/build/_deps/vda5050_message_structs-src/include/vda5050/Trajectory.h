// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_TRAJECTORY_H_
#define INCLUDE_VDA5050_TRAJECTORY_H_

#include <nlohmann/json.hpp>
#include <vector>

#include "vda5050/ControlPoint.h"

namespace vda5050 {

/// Points defining a spline. Theta allows holonomic vehicles to rotate along the trajecotry.
struct Trajectory {
  /// Range: [1 … float64.max]
  /// Degree of the NURBS curve defining the trajectory.
  /// If not defined, the default value is 1.
  double degree = 1.0;

  /// Range: [0.0 ... 1.0]
  /// Array of knot values of the NURBS.
  /// knotVector has size of number of control points + degree + 1.
  std::vector<double> knotVector;

  /// Array of controlPoint objects defining the control points of the NURBS,
  /// explicitly including the start and end point.
  std::vector<ControlPoint> controlPoints;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Trajectory &other) const {
    if (this->degree != other.degree) return false;
    if (this->knotVector != other.knotVector) return false;
    if (this->controlPoints != other.controlPoints) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Trajectory &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Trajectory &d);
void from_json(const json &j, Trajectory &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_TRAJECTORY_H_
