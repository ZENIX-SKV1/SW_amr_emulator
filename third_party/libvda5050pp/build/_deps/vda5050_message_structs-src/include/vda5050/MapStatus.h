// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#ifndef INCLUDE_VDA5050_MAPSTATUS_H_
#define INCLUDE_VDA5050_MAPSTATUS_H_

namespace vda5050 {

enum class MapStatus {
  /// Indicates this map version is currently not enabled on the AGV and thus
  /// could be enabled or deleted by request.
  DISABLED = 0,
  /// Indicates this map is currently active / used on the AGV. At most one map
  /// with the same mapId can have its status set to 'ENABLED'.
  ENABLED = 1,
};

}

#endif  // INCLUDE_VDA5050_MAPSTATUS_H_
