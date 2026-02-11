//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PUBLIC_VDA5050_2B_2B_MISC_ACTION_PARAMETER_VIEW_H_
#define PUBLIC_VDA5050_2B_2B_MISC_ACTION_PARAMETER_VIEW_H_

#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "vda5050/ActionParameter.h"

namespace vda5050pp::misc {

///
///\brief A convenience view for vda5050::ActionParameter vectors.
/// It can be used if you want to parse parameters by yourself.
///
class ActionParameterView {
private:
  const std::vector<vda5050::ActionParameter> &parameters_;

public:
  ///
  ///\brief Create a new view from a vector of ActionParameters.
  ///
  ///\param parameters the action parameters to view.
  ///
  explicit ActionParameterView(const std::vector<vda5050::ActionParameter> &parameters) noexcept(
      false);

  ///
  ///\brief Get the key's value as json-node
  ///
  ///\param key the parameter key
  ///\return vda5050::json the json-node
  ///\throws VDA5050PPInvalidActionParameterKey if the key is unknown.
  ///
  vda5050::json get(std::string_view key) const noexcept(false);

  ///
  ///\brief Try to get the key's value as json-node
  ///
  ///\param key the parameter key
  ///\return std::optional<vda5050::json> an optional json-node (if present)
  ///
  std::optional<vda5050::json> tryGet(std::string_view key) const;

  ///
  ///\brief Get the key's value as string
  ///
  ///\param key the parameter key
  ///\return std::string the key's value
  ///\throws VDA5050PPInvalidActionParameterKey if the key is unknown.
  ///
  std::string getString(std::string_view key) const noexcept(false);

  ///
  ///\brief Try to get the key's value as string
  ///
  ///\param key the parameter key
  ///\return std::optional<std::string> the key's value (if present)
  ///
  std::optional<std::string> tryGetString(std::string_view key) const;

  ///
  ///\brief Get the key's value as integer.
  ///
  ///\param key the parameter key
  ///\return int64_t the key's value
  ///\throws VDA5050PPInvalidActionParameterKey if the key is unknown.
  ///\throws VDA5050PPInvalidActionParameterType if the key cannot be parsed as int.
  ///
  int64_t getInt(std::string_view key) const noexcept(false);

  ///
  ///\brief Try to get the key's value as integer.
  ///
  ///\param key the parameter key
  ///\return std::optional<int64_t> the key's value (if present and parsable)
  ///
  std::optional<int64_t> tryGetInt(std::string_view key) const;

  ///
  ///\brief Get the key's value as double.
  ///
  ///\param key the parameter key
  ///\return double the key's value
  ///\throws VDA5050PPInvalidActionParameterKey if the key is unknown.
  ///\throws VDA5050PPInvalidActionParameterType if the key cannot be parsed as double.
  ///
  double getFloat(std::string_view key) const noexcept(false);

  ///
  ///\brief Try to get the key's value as double.
  ///
  ///\param key the parameter key
  ///\return std::optional<double> the key's value (if present and parsable)
  ///
  std::optional<double> tryGetFloat(std::string_view key) const;

  ///
  ///\brief Get the key's value as bool.
  ///
  ///\param key the parameter key
  ///\return bool the key's value
  ///\throws VDA5050PPInvalidActionParameterKey if the key is unknown.
  ///\throws VDA5050PPInvalidActionParameterType if the key cannot be parsed as bool.
  ///
  bool getBool(std::string_view key) const noexcept(false);

  ///
  ///\brief Try to get the key's value as bool.
  ///
  ///\param key the parameter key
  ///\return std::optional<bool> the key's value (if present and parsable)
  ///
  std::optional<bool> tryGetBool(std::string_view key) const;
};

}  // namespace vda5050pp::misc

#endif  // PUBLIC_VDA5050_2B_2B_MISC_ACTION_PARAMETER_VIEW_H_
