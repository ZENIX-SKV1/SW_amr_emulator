// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef VDA5050_SAFETYSTATE_H_
#define VDA5050_SAFETYSTATE_H_

#include <nlohmann/json.hpp>

#include "vda5050/EStop.h"

namespace vda5050 {
/// VD(M)A 5050 SafetyState
struct SafetyState {
  /// Emegerncy-Stop status
  EStop eStop = EStop::NONE;

  /// Protective field violation
  /// True: field is violated
  /// False: field is not violated
  bool fieldViolation = false;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const SafetyState &other) const {
    if (this->eStop != other.eStop) return false;
    if (this->fieldViolation != other.fieldViolation) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const SafetyState &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const SafetyState &d);
void from_json(const json &j, SafetyState &d);

}  // namespace vda5050
#endif  // VDA5050_SAFETYSTATE_H_
