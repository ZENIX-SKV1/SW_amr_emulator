// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ERRORLEVEL_H_
#define INCLUDE_VDA5050_ERRORLEVEL_H_

#include <ostream>
#include <nlohmann/json.hpp>

namespace vda5050 {
enum class ErrorLevel {

  /// warning: AGV is ready to start (e.g. maintenance
  ///          cycle expiration warning)
  WARNING,
  /// fatal: AGV is not in running condition, user
  ///        intervention required (e.g. laser scanner
  ///        is contaminated)
  FATAL
};

///
///\brief Write the enum-value to an ostream
///
///\param os the stream
///\param error_level the enum
///\return constexpr std::ostream&
///
constexpr std::ostream &operator<<(std::ostream &os, const ErrorLevel &error_level) {
  switch (error_level) {
    case ErrorLevel::WARNING:
      os << "WARNING";
      break;
    case ErrorLevel::FATAL:
      os << "FATAl";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

using json = nlohmann::json;
void to_json(json &j, const ErrorLevel &d);
void from_json(const json &j, ErrorLevel &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ERRORLEVEL_H_
