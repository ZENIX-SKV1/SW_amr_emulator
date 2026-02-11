// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef INCLUDE_VDA5050_WHEELTYPE_H
#define INCLUDE_VDA5050_WHEELTYPE_H

#include <nlohmann/json.hpp>

namespace vda5050 {

enum class WheelType { DRIVE, CASTER, FIXED, MECANUM };

using json = nlohmann::json;
void to_json(json &j, const WheelType &d);
void from_json(const json &j, WheelType &d);

} // namespace vda5050
#endif // INCLUDE_VDA5050_WHEELTYPE_H