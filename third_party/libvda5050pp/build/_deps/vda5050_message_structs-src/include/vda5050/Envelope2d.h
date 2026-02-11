// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_ENVELOPE2D_H_
#define VDA5050_ENVELOPE2D_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "vda5050/PolygonPoint.h"

namespace vda5050 {

struct Envelope2d {
  /// Name of the envelope curve set.
  std::string set;

  /// Envelope curve as a x/y-polygon polygon is assumed
  /// as closed and must be non-self-intersecting.
  std::vector<PolygonPoint> polygonPoints;

  /// Free-form text: description of envelope curve set.
  std::optional<std::string> description;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Envelope2d &other) const {
    if (this->set != other.set) return false;
    if (this->polygonPoints != other.polygonPoints) return false;
    if (this->description != other.description) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Envelope2d &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Envelope2d &d);
void from_json(const json &j, Envelope2d &d);

}  // namespace vda5050
#endif  // VDA5050_ENVELOPE2D_H_
