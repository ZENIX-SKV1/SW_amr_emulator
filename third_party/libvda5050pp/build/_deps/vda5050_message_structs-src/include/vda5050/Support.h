// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef INCLUDE_VDA5050_SUPPORT_H_
#define INCLUDE_VDA5050_SUPPORT_H_

#include <nlohmann/json.hpp>

namespace vda5050 {

enum class Support { SUPPORTED, REQUIRED };

using json = nlohmann::json;
void to_json(json &j, const Support &d);
void from_json(const json &j, Support &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_SUPPORT_H_
