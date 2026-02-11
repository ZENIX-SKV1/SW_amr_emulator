// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#include "vda5050++/core/agv_handler/distance_node_handler.h"

#include "vda5050++/config/distance_node_subconfig.h"
#include "vda5050++/core/instance.h"

using namespace vda5050pp::core::agv_handler;

void DistanceNodeHandler::handleNavigationStatusPosition(
    std::shared_ptr<vda5050pp::events::NavigationStatusPosition> evt) {
  if (evt == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("NavigationStatusPosition is nullptr"));
  }

  auto interpolation_type =
      Instance::ref().getConfig().getDistanceNodeSubConfig().getInterpolationType().value_or(
          vda5050pp::config::DistanceNodeSubConfig::NONE);

  switch (interpolation_type) {
    case vda5050pp::config::DistanceNodeSubConfig::NONE:
      break;
    case vda5050pp::config::DistanceNodeSubConfig::LINEAR: {
      vec_2_t new_pos{evt->position.x, evt->position.y};

      if (last_position_) {
        vec_2_t diff{new_pos.x - last_position_->x, new_pos.y - last_position_->y};
        distance_sum_ += std::sqrt(diff.x * diff.x + diff.y * diff.y);

        auto new_event =
            std::make_shared<vda5050pp::events::NavigationStatusDistanceSinceLastNode>();
        new_event->distance_since_last_node = distance_sum_;
        Instance::ref().getNavigationStatusManager().dispatch(new_event);
      }

      last_position_ = new_pos;
    } break;
    default:
      throw vda5050pp::VDA5050PPNotImplementedError(MK_EX_CONTEXT("Unknown interpolation type"));
  }
}

void DistanceNodeHandler::handleNavigationStatusNodeReached(
    std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached> evt) {
  if (evt == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(
        MK_EX_CONTEXT("NavigationStatusNodeReached is nullptr"));
  }

  auto interpolation_type =
      Instance::ref().getConfig().getDistanceNodeSubConfig().getInterpolationType().value_or(
          vda5050pp::config::DistanceNodeSubConfig::NONE);

  switch (interpolation_type) {
    case vda5050pp::config::DistanceNodeSubConfig::NONE:
      break;
    case vda5050pp::config::DistanceNodeSubConfig::LINEAR:
      distance_sum_ = 0.0;
      {
        auto new_event =
            std::make_shared<vda5050pp::events::NavigationStatusDistanceSinceLastNode>();
        new_event->distance_since_last_node = distance_sum_;
        Instance::ref().getNavigationStatusManager().dispatch(new_event);
      }
      break;
    default:
      throw vda5050pp::VDA5050PPNotImplementedError(MK_EX_CONTEXT("Unknown interpolation type"));
  }
}

void DistanceNodeHandler::initialize(Instance &instance) {
  this->navigation_status_subscriber_ =
      instance.getNavigationStatusManager().getScopedNavigationStatusSubscriber();

  this->navigation_status_subscriber_->subscribe(
      std::bind(std::mem_fn(&DistanceNodeHandler::handleNavigationStatusPosition), this,
                std::placeholders::_1));

  this->navigation_status_subscriber_->subscribe(
      std::bind(std::mem_fn(&DistanceNodeHandler::handleNavigationStatusNodeReached), this,
                std::placeholders::_1));

  distance_sum_ = 0.0;
  last_position_ = std::nullopt;
}

void DistanceNodeHandler::deinitialize(Instance &) { this->navigation_status_subscriber_.reset(); }

std::string_view DistanceNodeHandler::describe() const { return "DistanceNodeHandler"; }

std::shared_ptr<vda5050pp::config::ModuleSubConfig> DistanceNodeHandler::generateSubConfig() const {
  return std::make_shared<vda5050pp::config::DistanceNodeSubConfig>();
}