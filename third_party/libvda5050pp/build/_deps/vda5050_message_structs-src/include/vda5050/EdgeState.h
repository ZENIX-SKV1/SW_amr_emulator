// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_EDGESTATE_H_
#define INCLUDE_VDA5050_EDGESTATE_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/Trajectory.h"

namespace vda5050 {
/// VD(M)A 5050 EdgeState
struct EdgeState {
  /// Unique edge identification
  std::string edgeId;

  /// sequenceId to differentiate between multiple edges with
  /// the same edgeId
  uint32_t sequenceId = 0;

  /// Additional information on the edge
  std::optional<std::string> edgeDescription;

  /// True indicates that the edge is part of the base.
  /// False indicates that the edge is part of the horizon.
  bool released = false;

  /// The trajectory is to be communicated as a NURBS and is defined
  /// in chapter 6.4 Trajectory segments are from the point where
  /// the AGV starts to enter the edge until the point where it
  /// reports that the next node was traversed.
  std::optional<Trajectory> trajectory;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const EdgeState &other) const {
    if (this->edgeId != other.edgeId) return false;
    if (this->sequenceId != other.sequenceId) return false;
    if (this->edgeDescription != other.edgeDescription) return false;
    if (this->released != other.released) return false;
    if (this->trajectory != other.trajectory) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const EdgeState &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const EdgeState &d);
void from_json(const json &j, EdgeState &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_EDGESTATE_H_
