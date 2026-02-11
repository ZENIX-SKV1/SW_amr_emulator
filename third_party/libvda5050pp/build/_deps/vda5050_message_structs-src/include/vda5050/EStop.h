// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ESTOP_H_
#define INCLUDE_VDA5050_ESTOP_H_

#include <ostream>
#include <nlohmann/json.hpp>

namespace vda5050 {
/// Acknowledge-Type of eStop
enum class EStop {
  /// autoAck: autoacknowledgeable e-stop is activated e.g. by bumper or
  ///          protective field
  AUTOACK,

  /// manual: e-stop has to be acknowledged manually at the vehicle
  MANUAL,

  /// remote: facility e-stop has to be acknowledged remotely
  REMOTE,

  /// none: no e-stop activated
  NONE
};

///
///\brief Write the enum-value to an ostream
///
///\param os the stream
///\param e_stop the enum
///\return constexpr std::ostream&
///
constexpr std::ostream &operator<<(std::ostream &os, const EStop &e_stop) {
  switch (e_stop) {
    case EStop::AUTOACK:
      os << "AUTOACK";
      break;
    case EStop::MANUAL:
      os << "MANUAL";
      break;
    case EStop::REMOTE:
      os << "REMOTE";
      break;
    case EStop::NONE:
      os << "NONE";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

using json = nlohmann::json;
void to_json(json &j, const EStop &d);
void from_json(const json &j, EStop &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ESTOP_H_
