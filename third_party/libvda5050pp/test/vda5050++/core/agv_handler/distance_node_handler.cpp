// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#include <catch2/catch_all.hpp>

#include "vda5050++/core/events/event_control_blocks.h"
#include "vda5050++/core/instance.h"

static inline bool is_approx_equal(double a, double b, double epsilon = 1e-6) {
  return std::abs(a - b) <= epsilon;
}

TEST_CASE("core::agv_handler::DistanceNodeHandler", "[core][agv_handler]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_distance_node_handler);
  cfg.refDistanceNodeSubConfig().setInterpolationType(
      vda5050pp::config::DistanceNodeSubConfig::LINEAR);
  cfg.load(std::string_view("[module.DistanceNodeHandler]\n"
                            "distance_interpolation_type = 1"));
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  WHEN("[InterpolationType = LINEAR] Two position updates are dispatched") {
    bool evt_dispatched = false;
    double evt_dist = -1.0;

    auto sub = instance->getNavigationStatusManager().getScopedNavigationStatusSubscriber();
    sub.subscribe(
        [&evt_dispatched,
         &evt_dist](std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode> evt) {
          evt_dispatched = true;
          evt_dist = evt->distance_since_last_node;
        });

    auto evt1 = std::make_shared<vda5050pp::events::NavigationStatusPosition>();
    evt1->position.x = 1.5;
    evt1->position.y = -1.0;

    instance->getNavigationStatusManager().dispatch(evt1);

    THEN("A NavigationStatusDistanceSinceLastNode event is not dispatched for the first position "
         "update") {
      REQUIRE_FALSE(evt_dispatched);
    }

    evt_dispatched = false;
    evt_dist = -1.0;

    auto evt2 = std::make_shared<vda5050pp::events::NavigationStatusPosition>();
    evt2->position.x = -1.5;
    evt2->position.y = 3.0;

    instance->getNavigationStatusManager().dispatch(evt2);

    THEN("A NavigationStatusDistanceSinceLastNode event is dispatched for the second position "
         "update") {
      REQUIRE(evt_dispatched);
    }

    THEN("The distanceToLastNode is correct after the second update") {
      REQUIRE(is_approx_equal(evt_dist, 5.0));
    }
  }

  WHEN("[InterpolationType = LINEAR] A NavigationStatusNodeReached is dispatched") {
    bool evt_dispatched = false;
    double evt_dist = -1.0;

    auto sub = instance->getNavigationStatusManager().getScopedNavigationStatusSubscriber();
    sub.subscribe(
        [&evt_dispatched,
         &evt_dist](std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode> evt) {
          evt_dispatched = true;
          evt_dist = evt->distance_since_last_node;
        });

    auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();

    instance->getNavigationStatusManager().dispatch(evt);

    THEN("A NavigationStatusDistanceSinceLastNode event is dispatched") { REQUIRE(evt_dispatched); }

    THEN("The distanceToLastNode is zero") { REQUIRE(is_approx_equal(evt_dist, 0.0)); }
  }

  cfg.refDistanceNodeSubConfig().setInterpolationType(
      vda5050pp::config::DistanceNodeSubConfig::NONE);
  cfg.load(std::string_view("[module.DistanceNodeHandler]\n"
                            "distance_interpolation_type = 0"));
  vda5050pp::core::Instance::reset();
  instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  WHEN("[InterpolationType = NONE] Two position updates are dispatched") {
    bool evt_dispatched = false;

    auto sub = instance->getNavigationStatusManager().getScopedNavigationStatusSubscriber();
    sub.subscribe(
        [&evt_dispatched](
            std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode> evt) {
          evt_dispatched = true;
        });

    auto evt1 = std::make_shared<vda5050pp::events::NavigationStatusPosition>();
    evt1->position.x = 1.5;
    evt1->position.y = -1.0;

    instance->getNavigationStatusManager().dispatch(evt1);

    THEN("A NavigationStatusDistanceSinceLastNode event is not dispatched for the first position "
         "update") {
      REQUIRE_FALSE(evt_dispatched);
    }

    evt_dispatched = false;

    auto evt2 = std::make_shared<vda5050pp::events::NavigationStatusPosition>();
    evt2->position.x = -1.5;
    evt2->position.y = 3.0;

    instance->getNavigationStatusManager().dispatch(evt2);

    THEN("A NavigationStatusDistanceSinceLastNode event is not dispatched for the second position "
         "update") {
      REQUIRE_FALSE(evt_dispatched);
    }
  }

  WHEN("[InterpolationType = NONE] A NavigationStatusNodeReached is dispatched") {
    bool evt_dispatched = false;

    auto sub = instance->getNavigationStatusManager().getScopedNavigationStatusSubscriber();
    sub.subscribe(
        [&evt_dispatched](
            std::shared_ptr<vda5050pp::events::NavigationStatusDistanceSinceLastNode> evt) {
          evt_dispatched = true;
        });

    auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();

    instance->getNavigationStatusManager().dispatch(evt);

    THEN("A NavigationStatusDistanceSinceLastNode event is not dispatched") {
      REQUIRE_FALSE(evt_dispatched);
    }
  }
}