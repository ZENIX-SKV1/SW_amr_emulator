// Copyright Open Logistics Foundation
// 
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
// 
// 

#ifndef VDA5050_ACTIONSCOPE_H_
#define VDA5050_ACTIONSCOPE_H_

#include <nlohmann/json.hpp>

namespace vda5050 {

enum class ActionScope {
  /// usable as instantAction
  INSTANT,
  /// usable on nodes
  NODE,
  /// usable on edges
  EDGE
};

using json = nlohmann::json;
void to_json(json &j, const ActionScope &d);
void from_json(const json &j, ActionScope &d);

} // namespace vda5050
#endif // VDA5050_ACTIONSCOPE_H_
