// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_ENVELOPE3D_H_
#define VDA5050_ENVELOPE3D_H_

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace vda5050 {

struct Envelope3d {
  /// Name of the envelope curve set.
  std::string set;

  /// Format of data, e.g., DXF.
  std::string format;

  /// 3D-envelope curve data, format specified in 'format'.
  json data;

  /// Protocol and url-definition for downloading the 3D-envelope
  /// curve data, e.g. ftp://xxx.yyy.com/ac4dgvhoif5tghji.
  std::optional<std::string> url;

  /// Free-form text: description of envelope curve set
  std::optional<std::string> description;

  ///
  ///\brief Equality operator
  ///
  ///\param other the other object to compare to
  ///\return is equal?
  ///
  inline bool operator==(const Envelope3d &other) const {
    if (this->set != other.set) return false;
    if (this->format != other.format) return false;
    if (this->data != other.data) return false;
    if (this->url != other.url) return false;
    if (this->description != other.description) return false;

    return true;
  }

  ///
  ///\brief Inequality operator
  ///
  ///\param other the other object to compare to
  ///\return is not equal?
  ///
  inline bool operator!=(const Envelope3d &other) const { return !this->operator==(other); }
};

using json = nlohmann::json;
void to_json(json &j, const Envelope3d &d);
void from_json(const json &j, Envelope3d &d);

}  // namespace vda5050
#endif  // VDA5050_ENVELOPE3D_H_
