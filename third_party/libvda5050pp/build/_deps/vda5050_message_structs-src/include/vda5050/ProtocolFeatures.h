// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_PROTOCOLFEATURES_H_
#define VDA5050_PROTOCOLFEATURES_H_

#include <nlohmann/json.hpp>
#include <vector>

#include "vda5050/AgvAction.h"
#include "vda5050/OptionalParameter.h"

namespace vda5050 {

struct ProtocolFeatures {
  /// List of supported and/or required optional parameters.
  /// Optional parameters, that are not listed here, are assumed
  /// to be not supported by the AGV.
  std::vector<OptionalParameter> optionalParameters;

  /// List of all actions with parameters supported by this AGV.
  /// This includes standard actions specified in VDA5050 and
  /// manufacturer-specific actions.
  std::vector<AgvAction> agvActions;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const ProtocolFeatures &other) const {
    if (this->optionalParameters != other.optionalParameters) return false;
    if (this->agvActions != other.agvActions) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const ProtocolFeatures &other) const {
    return !this->operator==(other);
  }
};

using json = nlohmann::json;
void to_json(json &j, const ProtocolFeatures &d);
void from_json(const json &j, ProtocolFeatures &d);

}  // namespace vda5050
#endif  // VDA5050_PROTOCOLFEATURES_H_
