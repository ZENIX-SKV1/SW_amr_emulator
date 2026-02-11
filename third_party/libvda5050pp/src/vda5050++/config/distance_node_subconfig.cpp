// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// TODO: Short description

#include "vda5050++/config/distance_node_subconfig.h"

#include "vda5050++/core/config.h"

using namespace vda5050pp::config;

void DistanceNodeSubConfig::getFrom(const ConstConfigNode &node) {
  this->ModuleSubConfig::getFrom(node);

  auto node_view = vda5050pp::core::config::ConstConfigNode::upcast(node).get();

  auto interpolation_type = node_view["distance_interpolation_type"].value<int>();
  if (interpolation_type) {
    switch (*interpolation_type) {
      case 0:
        this->interpolation_type_ = InterpolationType::NONE;
        break;
      case 1:
        this->interpolation_type_ = InterpolationType::LINEAR;
        break;
      default:
        throw vda5050pp::VDA5050PPTOMLError(
            MK_EX_CONTEXT("Unknown interpolation type " + std::to_string(*interpolation_type)));
    }
  } else
    this->interpolation_type_ = std::nullopt;
}

void DistanceNodeSubConfig::putTo(ConfigNode &node) const {
  this->ModuleSubConfig::putTo(node);

  auto table = vda5050pp::core::config::ConfigNode::upcast(node).get().as_table();

  if (this->interpolation_type_) {
    table->insert("distance_interpolation_type", (int)*this->interpolation_type_);
  }
}

void DistanceNodeSubConfig::setInterpolationType(std::optional<InterpolationType> new_value) {
  this->interpolation_type_ = new_value;
}

std::optional<DistanceNodeSubConfig::InterpolationType>
DistanceNodeSubConfig::getInterpolationType() const {
  return this->interpolation_type_;
}