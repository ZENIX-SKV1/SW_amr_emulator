// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_LOADDIMENSIONS_H_
#define INCLUDE_VDA5050_LOADDIMENSIONS_H_

#include <optional>
#include <nlohmann/json.hpp>

namespace vda5050 {
/// Dimensions of the load’s bounding box in meters.
struct LoadDimensions {
  /// [m] Absolute length of the load’s bounding box.
  double length = 0.0;

  /// [m] Absolute width of the load’s bounding box.
  double width = 0.0;

  /// [m] Absolute height of the load’s bounding box.
  /// Optional: Set value only if known.
  std::optional<double> height;

  ///
  ///\brief Check if two LoadDimensions are equal
  ///
  /// used to interact with stl
  ///\param other the LoadDimension to compare to
  ///\return true equal
  ///\return false not equal
  ///
  inline bool operator==(const LoadDimensions &other) const noexcept(true) {
    if (this->length != other.length) return false;
    if (this->width != other.width) return false;
    if (this->height != other.height) return false;
    return true;
  }

  ///
  ///\brief Check if two LoadDimensions are not equal
  ///
  /// used to interact with stl
  ///\param other the LoadDimension to compare to
  ///\return true not equal
  ///\return false equal
  ///
  inline bool operator!=(const LoadDimensions &other) const noexcept(true) {
    return !(this->operator==(other));
  }
};

using json = nlohmann::json;
void to_json(json &j, const LoadDimensions &d);
void from_json(const json &j, LoadDimensions &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_LOADDIMENSIONS_H_
