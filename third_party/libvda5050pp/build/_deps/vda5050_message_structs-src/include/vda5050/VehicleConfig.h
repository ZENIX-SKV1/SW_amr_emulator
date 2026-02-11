// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#ifndef INCLUDE_VDA5050_VEHICLECONFIG_H_
#define INCLUDE_VDA5050_VEHICLECONFIG_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

#include "vda5050/Network.h"
#include "vda5050/VersionInfo.h"

namespace vda5050 {
struct VehicleConfig {
  /// Array of key-value pair objects containing software and hardware information.
  std::optional<std::vector<VersionInfo>> versions;

  /// Information about the vehicle's network connection. The listed information
  /// shall not be updated while the vehicle is operating.
  std::optional<Network> network;

  ///
  ///\brief Compare two VehicleConfig objects
  ///
  ///\param other VehicleConfig object to compare with
  ///\return true if the objects are equal
  ///
  inline bool operator==(const VehicleConfig &other) const {
    if (versions != other.versions) return false;
    if (network != other.network) return false;
    return true;
  }

  ///
  ///\brief Compare two VehicleConfig objects
  ///
  ///\param other VehicleConfig object to compare with
  ///\return true if the objects are not equal
  ///
  inline bool operator!=(const VehicleConfig &other) const { return !(*this == other); }
};

using json = nlohmann::json;
void to_json(json &j, const VehicleConfig &d);
void from_json(const json &j, VehicleConfig &d);
};  // namespace vda5050

#endif  // INCLUDE_VDA5050_VEHICLECONFIG_H_
