// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//


#ifndef INCLUDE_VDA5050_ERROR_H_
#define INCLUDE_VDA5050_ERROR_H_

#include <optional>
#include <string>
#include <vector>

#include "vda5050/ErrorLevel.h"
#include "vda5050/ErrorReference.h"
#include <nlohmann/json.hpp>

namespace vda5050 {
/// VD(M)A 5050 Error
struct Error {
  /// Type/name of error
  std::string errorType;

  /// Array of references to identify the source of the
  /// error (e. g. headerId, orderId, actionId, ...).
  /// For additional information see
  /// best practice chapter 7
  std::optional<std::vector<ErrorReference>> errorReferences;

  /// Error description
  std::optional<std::string> errorDescription;

  /// Hint on how to approach or solve the reported error.
  std::optional<std::string> errorHint;

  /// Enum {warning, fatal}
  /// warning: AGV is ready to start (e.g. maintenance
  ///          cycle expiration warning)
  /// fatal: AGV is not in running condition, user
  ///        intervention required (e.g. laser scanner
  ///        is contaminated)
  ErrorLevel errorLevel = ErrorLevel::WARNING;

  bool operator==(const Error &other) const {
    if (errorType != other.errorType) return false;
    if (errorReferences != other.errorReferences) return false;
    if (errorDescription != other.errorDescription) return false;
    if (errorHint != other.errorHint) return false;
    if (errorLevel != other.errorLevel) return false;
    return true;
  }
  inline bool operator!=(const Error &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Error &d);
void from_json(const json &j, Error &d);

}  // namespace vda5050
#endif  // INCLUDE_VDA5050_ERROR_H_
