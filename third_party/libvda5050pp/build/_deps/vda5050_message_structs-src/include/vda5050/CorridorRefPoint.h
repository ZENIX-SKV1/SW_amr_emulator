// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#ifndef INCLUDE_VDA5050_CORRIDORREFPOINT_H_
#define INCLUDE_VDA5050_CORRIDORREFPOINT_H_

#include <nlohmann/json.hpp>

namespace vda5050 {
/// Defines whether the boundaries are valid for the kinematic center
/// or the contour of the vehicle. If not specified the boundaries are
/// valid to the vehicles kinematic center.
enum class CorridorRefPoint {
  KINEMATICCENTER = 0,
  CONTOUR = 1,
};

using json = nlohmann::json;
void to_json(json &j, const CorridorRefPoint &d);
void from_json(const json &j, CorridorRefPoint &d);

}  // namespace vda5050

#endif  // INCLUDE_VDA5050_CORRIDORREFPOINT_H_
