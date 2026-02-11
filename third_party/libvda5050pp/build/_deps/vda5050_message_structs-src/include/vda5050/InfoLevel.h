// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_INFOLEVEL_H_
#define INCLUDE_VDA5050_INFOLEVEL_H_

#include <ostream>
#include <nlohmann/json.hpp>

namespace vda5050 {
enum class InfoLevel {
  /// used for debugging
  DEBUG,

  /// used for visualization
  INFO
};

///
///\brief Write the enum-value to an ostream
///
///\param os the stream
///\param info_level the enum
///\return constexpr std::ostream&
///
constexpr std::ostream &operator<<(std::ostream &os, const InfoLevel &info_level) {
  switch (info_level) {
    case InfoLevel::DEBUG:
      os << "DEBUG";
      break;
    case InfoLevel::INFO:
      os << "INFO";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

using json = nlohmann::json;
void to_json(json &j, const InfoLevel &d);
void from_json(const json &j, InfoLevel &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_INFOLEVEL_H_
