// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_LOADSET_H_
#define VDA5050_LOADSET_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "vda5050/BoundingBoxReference.h"
#include "vda5050/LoadDimensions.h"

namespace vda5050 {

struct LoadSet {
  /// Unique name of the load set, e.g., DEFAULT, SET1, etc.
  std::string setName;

  /// Type of load, e.g., EPAL, XLT1200, etc.
  std::string loadType;

  /// List of load positions btw. load handling devices,
  /// this load-set is valid for.
  /// If this parameter does not exist or is empty,
  /// this load-set is valid for all load handling devices on this AGV.
  std::optional<std::vector<std::string>> loadPositions;

  /// Bounding box reference as defined in parameter loads[] in state-message.
  std::optional<BoundingBoxReference> boundingBoxReference;

  /// Load dimensions as defined in parameter loads[] in state-message.
  std::optional<LoadDimensions> loadDimensions;

  /// [kg], maximum weight of loadtype.
  std::optional<double> maxWeight;

  /// [m], minimum allowed height for handling of this load-type and –weight
  /// references to boundingBoxReference.
  std::optional<double> minLoadhandlingHeight;

  /// [m], maximum allowed height for handling of this load-type and –weight
  /// references to boundingBoxReference.
  std::optional<double> maxLoadhandlingHeight;

  /// [m], minimum allowed depth for this load-type and –weight
  /// references to boundingBoxReference.
  std::optional<double> minLoadhandlingDepth;

  /// [m], maximum allowed depth for this load-type and –weight
  /// references to boundingBoxReference.
  std::optional<double> maxLoadhandlingDepth;

  /// [rad], minimum allowed tilt for this load-type and –weight.
  std::optional<double> minLoadhandlingTilt;

  /// [rad], maximum allowed tilt for this load-type and –weight.
  std::optional<double> maxLoadhandlingTilt;

  /// [m/s], maximum allowed speed for this load-type and –weight.
  std::optional<double> agvSpeedLimit;

  /// [m/s²], maximum allowed acceleration for this load-type and –weight.
  std::optional<double> agvAccelerationLimit;

  /// [m/s²], maximum allowed deceleration for this load-type and –weight.
  std::optional<double> agvDecelerationLimit;

  /// [s], approx. time for picking up the load
  std::optional<double> pickTime;

  /// [s], approx. time for dropping the load.
  std::optional<double> dropTime;

  /// Free-form text: description of the load handling set.
  std::optional<std::string> description;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const LoadSet &other) const {
    if (this->agvAccelerationLimit != other.agvAccelerationLimit) return false;
    if (this->agvDecelerationLimit != other.agvDecelerationLimit) return false;
    if (this->agvSpeedLimit != other.agvSpeedLimit) return false;
    if (this->boundingBoxReference != other.boundingBoxReference) return false;
    if (this->description != other.description) return false;
    if (this->dropTime != other.dropTime) return false;
    if (this->loadDimensions != other.loadDimensions) return false;
    if (this->loadPositions != other.loadPositions) return false;
    if (this->loadType != other.loadType) return false;
    if (this->maxLoadhandlingDepth != other.maxLoadhandlingDepth) return false;
    if (this->maxLoadhandlingHeight != other.maxLoadhandlingHeight) return false;
    if (this->maxLoadhandlingTilt != other.maxLoadhandlingTilt) return false;
    if (this->maxWeight != other.maxWeight) return false;
    if (this->minLoadhandlingDepth != other.minLoadhandlingDepth) return false;
    if (this->minLoadhandlingHeight != other.minLoadhandlingHeight) return false;
    if (this->minLoadhandlingTilt != other.minLoadhandlingTilt) return false;
    if (this->pickTime != other.pickTime) return false;
    if (this->setName != other.setName) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const LoadSet &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const LoadSet &d);
void from_json(const json &j, LoadSet &d);

}  // namespace vda5050
#endif  // VDA5050_LOADSET_H_
