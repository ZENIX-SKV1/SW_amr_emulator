// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#ifndef INCLUDE_VDA5050_MAP_H_
#define INCLUDE_VDA5050_MAP_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/MapStatus.h"

namespace vda5050 {

struct Map {
  /// ID of the map describing a defined area of the vehicle's workspace.
  std::string mapId;

  /// Version of the map.
  std::string mapVersion;

  /// Additional information on the map.
  std::optional<std::string> mapDescription;

  /// Enum {'ENABLED', 'DISABLED'}
  /// 'ENABLED': Indicates this map is currently active / used on the AGV.
  /// At most one map with the same mapId can have its status set to 'ENABLED'.
  /// 'DISABLED': Indicates this map version is currently not enabled on the AGV
  /// and thus could be enabled or deleted by request.
  MapStatus mapStatus = MapStatus::DISABLED;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Map &other) const {
    if (this->mapId != other.mapId) return false;
    if (this->mapVersion != other.mapVersion) return false;
    if (this->mapDescription != other.mapDescription) return false;
    if (this->mapStatus != other.mapStatus) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Map &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Map &d);
void from_json(const json &j, Map &d);

}  // namespace vda5050

#endif  // INCLUDE_VDA5050_MAP_H_
