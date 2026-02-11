// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_OPTIONALPARAMETER_H_
#define VDA5050_OPTIONALPARAMETER_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/Support.h"

namespace vda5050 {

struct OptionalParameter {
  /// Full name of optional parameter, e.g. “order.nodes.nodePosition.allowedDeviationTheta”.
  std::string parameter;

  /// Type of support for the optional parameter, the following values are possible:
  /// SUPPORTED: optional parameter is supported like specified.
  /// REQUIRED: optional parameter is required for proper AGV-operation.
  Support support = Support::SUPPORTED;

  // Free-form text: description of optional parameter, e.g.:
  // - Reason, why the optional parameter ‘direction’ is necessary for this AGV-type and which
  // values it can contain.
  // - The parameter ‘nodeMarker’ must contain unsigned interger-numbers only.
  // - NURBS-Support is limited to straight lines and circle segments.
  std::optional<std::string> description;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const OptionalParameter &other) const {
    if (this->parameter != other.parameter) return false;
    if (this->description != other.description) return false;
    if (this->support != other.support) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const OptionalParameter &other) const {
    return !this->operator==(other);
  }
};

using json = nlohmann::json;
void to_json(json &j, const OptionalParameter &d);
void from_json(const json &j, OptionalParameter &d);

}  // namespace vda5050
#endif  // VDA5050_OPTIONALPARAMETER_H_
