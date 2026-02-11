// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ORIENTATIONTYPE_H_
#define INCLUDE_VDA5050_ORIENTATIONTYPE_H_

#include <cstdint>
#include <ostream>
#include <nlohmann/json.hpp>

namespace vda5050 {

enum class OrientationType : uint8_t {
  /// tangential to the edge.
  TANGENTIAL,

  /// relative to the global project specific map coordinate system
  GLOBAL
};

///
///\brief Write the enums value-name to an ostream
///
///\param os the stream
///\param operating_mode the enum
///\return constexpr std::ostream&
///
constexpr std::ostream &operator<<(std::ostream &os, const OrientationType &orientation_type) {
  switch (orientation_type) {
    case OrientationType::TANGENTIAL:
      os << "TANGENTIAL";
      break;
    case OrientationType::GLOBAL:
      os << "GLOBAL";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

using json = nlohmann::json;
void to_json(json &j, const OrientationType &d);
void from_json(const json &j, OrientationType &d);

}

#endif // INCLUDE_VDA5050_ORIENTATIONTYPE_H_
