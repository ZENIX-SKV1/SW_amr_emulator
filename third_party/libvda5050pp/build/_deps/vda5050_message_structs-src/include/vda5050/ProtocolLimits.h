// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_PROTOCOLLIMITS_H_
#define VDA5050_PROTOCOLLIMITS_H_

#include <nlohmann/json.hpp>

#include "vda5050/MaxArrayLens.h"
#include "vda5050/MaxStringLens.h"
#include "vda5050/Timing.h"

namespace vda5050 {

struct ProtocolLimits {
  /// Maximum lengths of strings.
  MaxStringLens maxStringLens;

  /// Maximum lengths of arrays.
  MaxArrayLens maxArrayLens;

  /// Timing information.
  Timing timing;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const ProtocolLimits &other) const {
    if (this->maxStringLens != other.maxStringLens) return false;
    if (this->maxArrayLens != other.maxArrayLens) return false;
    if (this->timing != other.timing) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const ProtocolLimits &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const ProtocolLimits &d);
void from_json(const json &j, ProtocolLimits &d);

}  // namespace vda5050
#endif  // VDA5050_PROTOCOLLIMITS_H_
