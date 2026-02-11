// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include <catch2/catch_all.hpp>

#include "test/data.h"
#include "vda5050++/core/instance.h"
#include "vda5050++/observer/order_observer.h"

TEST_CASE("CancelOrder with only horizon", "[cancelOrder][high_level]") {
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_interpreter_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_message_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_navigation_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_order_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_query_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_state_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_validation_event_handler_key);
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  vda5050pp::core::events::EventManagerOptions event_manager_options;
  event_manager_options.synchronous_event_dispatch = true;
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, event_manager_options).lock();

  vda5050pp::observer::OrderObserver order_observer;

  GIVEN("An order with a horizon was received and processed until the base is empty") {
    auto valid_order = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
    valid_order->valid_order = std::make_shared<vda5050::Order>(test::data::mkTemplateOrder({
        test::data::TemplateElement{"n1", 0, true, {}},
        test::data::TemplateElement{"e1", 1, true, {}},
        test::data::TemplateElement{"n2", 2, true, {}},
        test::data::TemplateElement{"e2", 3, false, {}},
        test::data::TemplateElement{"n3", 4, false, {}},
    }));
    valid_order->valid_order->orderId = "order1";
    valid_order->valid_order->orderUpdateId = 0;

    instance->getMessageEventManager().synchronousDispatch(valid_order);

    const auto &graph = instance->getOrderManager().getCurrentGraph();
    REQUIRE(graph.baseBounds().first == 0);
    REQUIRE(graph.baseBounds().second == 2);
    REQUIRE(graph.horizonBounds().first == 3);
    REQUIRE(graph.horizonBounds().second == 4);
    REQUIRE(order_observer.getOrderStatus() == vda5050pp::misc::OrderStatus::k_order_active);

    auto reach_2 = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
    reach_2->last_node_id = "n1";
    reach_2->node_seq_id = 2;

    instance->getNavigationStatusManager().dispatch(reach_2);

    REQUIRE(graph.baseBounds().first == 2);
    REQUIRE(graph.baseBounds().second == 2);
    REQUIRE(graph.horizonBounds().first == 3);
    REQUIRE(graph.horizonBounds().second == 4);
    REQUIRE(order_observer.getOrderStatus() == vda5050pp::misc::OrderStatus::k_order_idle);

    WHEN("A cancelOrder instant action is received") {
      auto cancel_order =
          std::make_shared<vda5050pp::core::events::ReceiveInstantActionMessageEvent>();
      cancel_order->instant_actions = std::make_shared<vda5050::InstantActions>();
      cancel_order->instant_actions->actions = {
          test::data::mkAction("id1", "cancelOrder", vda5050::BlockingType::HARD),
      };

      instance->getMessageEventManager().synchronousDispatch(cancel_order);

      THEN("It is finished") {
        REQUIRE(instance->getOrderManager().getActionState("id1")->actionStatus ==
                vda5050::ActionStatus::FINISHED);
      }

      THEN("The scheduler is idle") {
        REQUIRE(order_observer.getOrderStatus().value() ==
                vda5050pp::misc::OrderStatus::k_order_idle);
      }

      THEN("The graph is cleared") { REQUIRE_FALSE(instance->getOrderManager().hasGraph()); }
    }
  }
}
