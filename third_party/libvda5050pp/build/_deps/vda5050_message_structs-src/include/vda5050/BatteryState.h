// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_BATTERYSTATE_H_
#define INCLUDE_VDA5050_BATTERYSTATE_H_

#include <cstdint>
#include <optional>
#include <nlohmann/json.hpp>

namespace vda5050 {
/// VD(M)A 5050 BatteryState
struct BatteryState {
  /// [%] State of Charge: if AGV only provides values for good or bad
  /// battery levels, these will be indicated as 20% (bad) and 80% (good).
  double batteryCharge = 0.0;

  /// [V] Battery Voltage
  std::optional<double> batteryVoltage;

  /// [%] Range: [0 ... 100]
  /// State of Health
  std::optional<int8_t> batteryHealth;

  /// True: charging in progress
  /// False: AGV is currently not charging
  bool charging = false;

  /// [m] Range: [0.0 ... uint32.max]
  /// Estimated reach with current Stage of Charge
  std::optional<uint32_t> reach;

  inline bool operator==(const BatteryState &other) const {
    if (batteryCharge != other.batteryCharge) return false;
    if (batteryVoltage != other.batteryVoltage) return false;
    if (batteryHealth != other.batteryHealth) return false;
    if (charging != other.charging) return false;
    if (reach != other.reach) return false;
    return true;
  }
  inline bool operator!=(const BatteryState &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const BatteryState &d);
void from_json(const json &j, BatteryState &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_BATTERYSTATE_H_
