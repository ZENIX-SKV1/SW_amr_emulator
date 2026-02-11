// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef INCLUDE_VDA5050_AGVFACTSHEET_H_
#define INCLUDE_VDA5050_AGVFACTSHEET_H_

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "vda5050/AgvGeometry.h"
#include "vda5050/Header_vda5050.h"
#include "vda5050/LoadSpecification.h"
#include "vda5050/PhysicalParameters.h"
#include "vda5050/ProtocolFeatures.h"
#include "vda5050/ProtocolLimits.h"
#include "vda5050/TypeSpecification.h"
#include "vda5050/VehicleConfig.h"
namespace vda5050 {

struct AgvFactsheet {
  /// Message header.
  vda5050::HeaderVDA5050 header;

  /// These parameters generally specify the class and the capabilities of the AGV.
  TypeSpecification typeSpecification;

  /// These parameters specify the basic physical properties of the AGV.
  PhysicalParameters physicalParameters;

  /// Limits for length of identifiers, arrays, strings and similar in MQTT communication.
  ProtocolLimits protocolLimits;

  /// Supported features of VDA5050 protocol.
  ProtocolFeatures protocolFeatures;

  /// Detailed definition of AGV geometry.
  AgvGeometry agvGeometry;

  /// Abstract specification of load capabilities.
  LoadSpecification loadSpecification;

  /// Summary of current software and hardware versions on the vehicle and optional
  /// network information.
  std::optional<VehicleConfig> vehicleConfig;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const AgvFactsheet &other) const {
    if (this->header != other.header) return false;
    if (this->typeSpecification != other.typeSpecification) return false;
    if (this->physicalParameters != other.physicalParameters) return false;
    if (this->protocolLimits != other.protocolLimits) return false;
    if (this->protocolFeatures != other.protocolFeatures) return false;
    if (this->agvGeometry != other.agvGeometry) return false;
    if (this->loadSpecification != other.loadSpecification) return false;
    if (this->vehicleConfig != other.vehicleConfig) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const AgvFactsheet &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const AgvFactsheet &d);
void from_json(const json &j, AgvFactsheet &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_AGVFACTSHEET_H_
