// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include <catch2/catch_all.hpp>

#include "test/data.h"
#include "vda5050++/core/instance.h"

TEST_CASE("Increasing an Order with a single-node-order", "[core][event][combined]") {
  vda5050pp::Config cfg;
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_state_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_interpreter_event_handler_key);
  vda5050pp::core::Instance::reset();
  vda5050pp::core::Instance::init(cfg, evt_opts);

  auto initial_order_no_horizon = test::data::wrap_shared(test::data::mkTemplateOrder({
      test::data::TemplateElement{"n0", 0, true, {}},
      test::data::TemplateElement{"e01", 1, true, {}},
      test::data::TemplateElement{"n1", 2, true, {}},
      test::data::TemplateElement{"e12", 3, true, {}},
      test::data::TemplateElement{"n2", 4, true, {}},
  }));
  initial_order_no_horizon->orderId = "order";
  initial_order_no_horizon->orderUpdateId = 0;

  auto horizon_update_order = test::data::wrap_shared(test::data::mkTemplateOrder({
      test::data::TemplateElement{"n2", 4, true, {}},
      test::data::TemplateElement{"e23", 5, false, {}},
      test::data::TemplateElement{"n3", 6, false, {}},
      test::data::TemplateElement{"e34", 7, false, {}},
      test::data::TemplateElement{"n4", 8, false, {}},
  }));
  horizon_update_order->orderId = "order";
  horizon_update_order->orderUpdateId = 1;

  WHEN("The an initial order without a horizon is received") {
    auto evt = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
    evt->valid_order = initial_order_no_horizon;
    vda5050pp::core::Instance::ref().getMessageEventManager().synchronousDispatch(evt);

    THEN("The state contains the order") {
      const auto &graph = vda5050pp::core::Instance::ref().getOrderManager().getCurrentGraph();
      REQUIRE(graph.baseBounds().first == 0);
      REQUIRE(graph.baseBounds().second == 4);
      REQUIRE_FALSE(graph.hasHorizon());
      REQUIRE(vda5050pp::core::Instance::ref().getOrderManager().getOrderId().first == "order");
      REQUIRE(vda5050pp::core::Instance::ref().getOrderManager().getOrderId().second == 0);
    }

    WHEN("The order is updated with a horizon") {
      auto evt = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
      evt->valid_order = horizon_update_order;
      vda5050pp::core::Instance::ref().getMessageEventManager().synchronousDispatch(evt);

      THEN("The state contains the order with the horizon") {
        const auto &graph = vda5050pp::core::Instance::ref().getOrderManager().getCurrentGraph();
        REQUIRE(graph.baseBounds().first == 0);
        REQUIRE(graph.baseBounds().second == 4);
        REQUIRE(graph.hasHorizon());
        REQUIRE(graph.horizonBounds().first == 5);
        REQUIRE(graph.horizonBounds().second == 8);
        REQUIRE(vda5050pp::core::Instance::ref().getOrderManager().getOrderId().first == "order");
        REQUIRE(vda5050pp::core::Instance::ref().getOrderManager().getOrderId().second == 1);
      }
    }
  }
}

TEST_CASE("[Public Issue #27] Initializing a node action") {
  vda5050pp::Config cfg;
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  cfg.refGlobalConfig().useBlackList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_mqtt_key);
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto test_order = test::data::wrap_shared(test::data::mkTemplateOrder({
      test::data::TemplateElement{"n0", 0, true, {}},
      test::data::TemplateElement{"e01", 1, true, {}},
      test::data::TemplateElement{"n1", 2, true, {}},
      test::data::TemplateElement{"e12", 3, true, {}},
      test::data::TemplateElement{
          "n2", 4, true, {{"test action", "a0", "", vda5050::BlockingType::NONE}}},
  }));
  test_order->orderId = "order";
  test_order->orderUpdateId = 0;

  WHEN("The order is received") {
    auto evt = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
    evt->valid_order = test_order;
    vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(evt);

    THEN("The action is WAITING") {
      REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
              vda5050::ActionStatus::WAITING);
    }

    WHEN("The node n1 is reached") {
      auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
      evt->last_node_id = "n1";
      evt->node_seq_id = 2;
      instance->getNavigationStatusManager().dispatch(evt);

      THEN("The action is WAITING") {
        REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                vda5050::ActionStatus::WAITING);
      }

      WHEN("The node n2 is reached") {
        auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
        evt->last_node_id = "n2";
        evt->node_seq_id = 4;
        instance->getNavigationStatusManager().dispatch(evt);

        THEN("The action is INITIALIZING") {
          REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);
        }
      }
    }
  }
}

TEST_CASE("Triggering multiple actions on nodes", "[core][event][combined]") {
  vda5050pp::Config cfg;
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  cfg.refGlobalConfig().useBlackList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_mqtt_key);
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto test_order = test::data::wrap_shared(test::data::mkTemplateOrder(
      {test::data::TemplateElement{
           "n0", 0, true, {{"test action", "a0", "", vda5050::BlockingType::NONE}}},
       test::data::TemplateElement{"e01", 1, true, {}},
       test::data::TemplateElement{"n1",
                                   2,
                                   true,
                                   {{"test action", "a1", "", vda5050::BlockingType::NONE},
                                    {"test action", "a2", "", vda5050::BlockingType::NONE}}},
       test::data::TemplateElement{"e12", 3, true, {}},
       test::data::TemplateElement{
           "n2", 4, true, {{"test action", "a3", "", vda5050::BlockingType::NONE}}},
       test::data::TemplateElement{"e23", 5, true, {}},
       test::data::TemplateElement{
           "n3", 6, true, {{"test action", "a4", "", vda5050::BlockingType::SOFT}}}}));
  test_order->orderId = "order";
  test_order->orderUpdateId = 0;

  WHEN("The order is received") {
    auto evt = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
    evt->valid_order = test_order;
    vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(evt);

    THEN("All actions are in the correct state") {
      REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
              vda5050::ActionStatus::INITIALIZING);

      REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
              vda5050::ActionStatus::WAITING);

      REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
              vda5050::ActionStatus::WAITING);

      REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
              vda5050::ActionStatus::WAITING);

      REQUIRE(instance->getOrderManager().getActionState("a4")->actionStatus ==
              vda5050::ActionStatus::WAITING);
    }

    WHEN("The node n1 is reached") {
      auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
      evt->last_node_id = "n1";
      evt->node_seq_id = 2;
      instance->getNavigationStatusManager().dispatch(evt);

      THEN("All actions are in the correct state") {
        REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                vda5050::ActionStatus::INITIALIZING);

        REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
                vda5050::ActionStatus::INITIALIZING);

        REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
                vda5050::ActionStatus::INITIALIZING);

        REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
                vda5050::ActionStatus::WAITING);

        REQUIRE(instance->getOrderManager().getActionState("a4")->actionStatus ==
                vda5050::ActionStatus::WAITING);
      }

      WHEN("The node n2 is reached") {
        auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
        evt->last_node_id = "n2";
        evt->node_seq_id = 4;
        instance->getNavigationStatusManager().dispatch(evt);

        THEN("All actions are in the correct state") {
          REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);

          REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);

          REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);

          REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);

          REQUIRE(instance->getOrderManager().getActionState("a4")->actionStatus ==
                  vda5050::ActionStatus::WAITING);
        }

        WHEN("The node n3 is reached") {
          auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
          evt->last_node_id = "n3";
          evt->node_seq_id = 6;
          instance->getNavigationStatusManager().dispatch(evt);

          THEN("All actions are in the correct state") {
            REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);

            REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);

            REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);

            REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);

            REQUIRE(instance->getOrderManager().getActionState("a4")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);
          }
        }
      }
    }
  }
}

TEST_CASE("Triggering multiple actions on edges", "[core][event][combined]") {
  vda5050pp::Config cfg;
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  cfg.refGlobalConfig().useBlackList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_mqtt_key);
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto test_order =
      test::data::wrap_shared(test::data::mkTemplateOrder(
          {test::data::TemplateElement{"n0", 0, true, {}},
           test::data::TemplateElement{
               "e01", 1, true, {{"test action", "a0", "", vda5050::BlockingType::NONE}}},
           test::data::TemplateElement{"n1", 2, true, {}},
           test::data::TemplateElement{"e12",
                                       3,
                                       true,
                                       {{"test action", "a1", "", vda5050::BlockingType::NONE},
                                        {"test action", "a2", "", vda5050::BlockingType::NONE}}},
           test::data::TemplateElement{"n2", 4, true, {}},
           test::data::TemplateElement{
               "e23", 5, true, {{"test action", "a3", "", vda5050::BlockingType::NONE}}},
           test::data::TemplateElement{"n3", 6, true, {}}}));
  test_order->orderId = "order";
  test_order->orderUpdateId = 0;

  WHEN("The order is received") {
    auto evt = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
    evt->valid_order = test_order;
    vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(evt);

    THEN("All actions are in the correct state") {
      REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
              vda5050::ActionStatus::INITIALIZING);

      REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
              vda5050::ActionStatus::WAITING);

      REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
              vda5050::ActionStatus::WAITING);

      REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
              vda5050::ActionStatus::WAITING);
    }

    WHEN("The node n1 is reached") {
      auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
      evt->last_node_id = "n1";
      evt->node_seq_id = 2;
      instance->getNavigationStatusManager().dispatch(evt);

      THEN("All actions are in the correct state") {
        REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                vda5050::ActionStatus::INITIALIZING);

        REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
                vda5050::ActionStatus::INITIALIZING);

        REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
                vda5050::ActionStatus::INITIALIZING);

        REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
                vda5050::ActionStatus::WAITING);
      }

      WHEN("The node n2 is reached") {
        auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
        evt->last_node_id = "n2";
        evt->node_seq_id = 4;
        instance->getNavigationStatusManager().dispatch(evt);

        THEN("All actions are in the correct state") {
          REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);

          REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);

          REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);

          REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
                  vda5050::ActionStatus::INITIALIZING);
        }

        WHEN("The node n3 is reached") {
          auto evt = std::make_shared<vda5050pp::events::NavigationStatusNodeReached>();
          evt->last_node_id = "n3";
          evt->node_seq_id = 6;
          instance->getNavigationStatusManager().dispatch(evt);

          THEN("All actions are in the correct state") {
            REQUIRE(instance->getOrderManager().getActionState("a0")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);

            REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);

            REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);

            REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
                    vda5050::ActionStatus::INITIALIZING);
          }
        }
      }
    }
  }
}