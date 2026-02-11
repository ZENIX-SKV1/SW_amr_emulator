//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/misc/action_parameter_view.h"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <set>

#include "vda5050++/core/common/container.h"
#include "vda5050++/core/common/exception.h"

using namespace vda5050pp::misc;

inline std::optional<vda5050::json> lookupActionParameter(
    const std::vector<vda5050::ActionParameter> &parameters, std::string_view key) noexcept(false) {
  auto match = [key](const vda5050::ActionParameter &parameter) { return parameter.key == key; };

  if (auto it = std::find_if(begin(parameters), end(parameters), match); it != end(parameters)) {
    return std::make_optional<vda5050::json>(it->value);
  } else {
    return std::nullopt;
  }
}

ActionParameterView::ActionParameterView(
    const std::vector<vda5050::ActionParameter> &parameters) noexcept(false)
    : parameters_(parameters) {}

vda5050::json ActionParameterView::get(std::string_view key) const noexcept(false) {
  if (auto maybe_value = lookupActionParameter(this->parameters_, key); maybe_value.has_value()) {
    return *maybe_value;
  } else {
    throw vda5050pp::VDA5050PPInvalidActionParameterKey(
        MK_EX_CONTEXT(fmt::format("Key \"{}\" not found", key)));
  }
}

std::optional<vda5050::json> ActionParameterView::tryGet(std::string_view key) const {
  return lookupActionParameter(this->parameters_, key);
}

std::string ActionParameterView::getString(std::string_view key) const noexcept(false) {
  if (auto maybe_value = lookupActionParameter(this->parameters_, key); maybe_value.has_value()) {
    if (maybe_value->is_string()) {
      return maybe_value->get<std::string>();
    } else {
      throw vda5050pp::VDA5050PPInvalidActionParameterType(
          MK_EX_CONTEXT(fmt::format("Key \"{}\" is not a string", key)));
    }
  } else {
    throw vda5050pp::VDA5050PPInvalidActionParameterKey(
        MK_EX_CONTEXT(fmt::format("Key \"{}\" not found", key)));
  }
}
std::optional<std::string> ActionParameterView::tryGetString(std::string_view key) const {
  if (auto maybe_value = lookupActionParameter(this->parameters_, key);
      maybe_value.has_value() && maybe_value->is_string()) {
    return maybe_value->get<std::string>();
  }

  return std::nullopt;
}

int64_t ActionParameterView::getInt(std::string_view key) const noexcept(false) {
  auto value = this->get(key);
  if (value.is_number_integer()) {
    return value.get<int64_t>();
  } else {
    throw vda5050pp::VDA5050PPInvalidActionParameterType(
        MK_EX_CONTEXT(fmt::format("Key \"{}\" is not an integer", key)));
  }
}
std::optional<int64_t> ActionParameterView::tryGetInt(std::string_view key) const {
  if (auto maybe_value = this->tryGet(key);
      maybe_value.has_value() && maybe_value->is_number_integer()) {
    return maybe_value->get<int64_t>();
  } else {
    return std::nullopt;
  }
}

double ActionParameterView::getFloat(std::string_view key) const noexcept(false) {
  auto value = this->get(key);
  if (value.is_number_float()) {
    return value.get<double>();
  } else {
    throw vda5050pp::VDA5050PPInvalidActionParameterType(
        MK_EX_CONTEXT(fmt::format("Key \"{}\" is not a float", key)));
  }
}
std::optional<double> ActionParameterView::tryGetFloat(std::string_view key) const {
  if (auto maybe_value = this->tryGet(key);
      maybe_value.has_value() && maybe_value->is_number_float()) {
    return maybe_value->get<double>();
  } else {
    return std::nullopt;
  }
}

bool ActionParameterView::getBool(std::string_view key) const noexcept(false) {
  auto value = this->get(key);
  if (value.is_boolean()) {
    return value.get<bool>();
  } else {
    throw vda5050pp::VDA5050PPInvalidActionParameterType(
        MK_EX_CONTEXT(fmt::format("Key \"{}\" is not a boolean", key)));
  }
}

std::optional<bool> ActionParameterView::tryGetBool(std::string_view key) const {
  if (auto maybe_value = this->tryGet(key); maybe_value.has_value() && maybe_value->is_boolean()) {
    return maybe_value->get<bool>();
  } else {
    return std::nullopt;
  }
}