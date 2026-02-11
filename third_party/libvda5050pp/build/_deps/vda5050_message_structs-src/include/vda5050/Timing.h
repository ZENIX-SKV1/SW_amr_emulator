// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_TIMING_H_
#define VDA5050_TIMING_H_

#include <nlohmann/json.hpp>
#include <optional>

namespace vda5050 {

struct Timing {
  /// [s], Minimum interval sending order messages to the AGV.
  float minOrderInterval = 0.0;

  /// [s], Minimum interval for sending state-messages.
  float minStateInterval = 0.0;

  /// [s], Default interval for sending state-messages,
  /// if not defined, the default value from the main document is used.
  std::optional<float> defaultStateInterval;

  /// [s], Default interval for sending messages on visualization topic.
  std::optional<float> visualizationInterval;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Timing &other) const {
    if (this->minOrderInterval != other.minOrderInterval) return false;
    if (this->minStateInterval != other.minStateInterval) return false;
    if (this->defaultStateInterval != other.defaultStateInterval) return false;
    if (this->visualizationInterval != other.visualizationInterval) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Timing &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Timing &d);
void from_json(const json &j, Timing &d);

}  // namespace vda5050
#endif  // VDA5050_TIMING_H_
