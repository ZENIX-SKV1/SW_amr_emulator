// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include "vda5050++/sinks/control_sink.h"

#include <catch2/catch_all.hpp>

#include "test/data.h"
#include "vda5050++/core/instance.h"
#include "vda5050++/observer/order_observer.h"

using namespace std::chrono_literals;

TEST_CASE("sinks::ControlSink - cancel", "[events][sinks]") {
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
  GIVEN("An order was received") {
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

    WHEN("A control sink initialtes a cancel action blockingly") {
      auto cancel_result = std::async(std::launch::async, [] {
        vda5050pp::sinks::ControlSink control_sink;
        control_sink.initiateCancel("Test caused cancel", true);
      });

      THEN("Before returning, the scheduler is not idle") {
        REQUIRE(order_observer.getOrderStatus().value() !=
                vda5050pp::misc::OrderStatus::k_order_idle);

        WHEN("The user canceled the order") {
          auto nav_failed = std::make_shared<vda5050pp::events::NavigationStatusControl>();
          nav_failed->type = vda5050pp::events::NavigationStatusControlType::k_failed;
          vda5050pp::core::Instance::ref().getNavigationStatusManager().dispatch(nav_failed);

          REQUIRE(cancel_result.wait_for(100ms) == std::future_status::ready);

          THEN("After returning, the scheduler is idle") {
            REQUIRE(order_observer.getOrderStatus().value() ==
                    vda5050pp::misc::OrderStatus::k_order_idle);
          }

          THEN("After returning, the graph is cleared") {
            REQUIRE_FALSE(instance->getOrderManager().hasGraph());
          }
        }
      }
    }

    WHEN("A control sink initialtes a cancel action non-blockingly") {
      auto cancel_result = std::async(std::launch::async, [] {
        vda5050pp::sinks::ControlSink control_sink;
        control_sink.initiateCancel("Test caused cancel", false);
      });

      THEN("The call returned immediately") {
        REQUIRE(cancel_result.wait_for(100ms) == std::future_status::ready);

        THEN("The scheduler is not idle") {
          REQUIRE(order_observer.getOrderStatus().value() !=
                  vda5050pp::misc::OrderStatus::k_order_idle);
        }

        WHEN("The user canceled the order") {
          auto nav_failed = std::make_shared<vda5050pp::events::NavigationStatusControl>();
          nav_failed->type = vda5050pp::events::NavigationStatusControlType::k_failed;
          vda5050pp::core::Instance::ref().getNavigationStatusManager().dispatch(nav_failed);

          THEN("The scheduler is idle and the graph is cleared") {
            REQUIRE(order_observer.getOrderStatus().value() ==
                    vda5050pp::misc::OrderStatus::k_order_idle);
            REQUIRE_FALSE(instance->getOrderManager().hasGraph());
          }
        }
      }
    }
  }
}

TEST_CASE("sinks::ControlSink - pause/resume", "[events][sinks]") {
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
  GIVEN("An order was received") {
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

    WHEN("A control sink initialtes a pause action blockingly") {
      auto pause_result = std::async(std::launch::async, [] {
        vda5050pp::sinks::ControlSink control_sink;
        control_sink.initiatePause("Test caused pause", true);
      });

      THEN("Before returning, the scheduler is not paused") {
        REQUIRE(order_observer.getOrderStatus().value() !=
                vda5050pp::misc::OrderStatus::k_order_paused);

        WHEN("The user paused the order") {
          auto nav_paused = std::make_shared<vda5050pp::events::NavigationStatusControl>();
          nav_paused->type = vda5050pp::events::NavigationStatusControlType::k_paused;
          vda5050pp::core::Instance::ref().getNavigationStatusManager().dispatch(nav_paused);

          REQUIRE(pause_result.wait_for(100ms) == std::future_status::ready);

          THEN("After returning, the scheduler is paused") {
            REQUIRE(order_observer.getOrderStatus().value() ==
                    vda5050pp::misc::OrderStatus::k_order_paused);

            WHEN("A control sink initialtes a resume action blockingly") {
              auto resume_result = std::async(std::launch::async, [] {
                vda5050pp::sinks::ControlSink control_sink;
                control_sink.initiateResume("Test Resumed resume", true);
              });

              THEN("Before returning, the scheduler is not active") {
                REQUIRE(order_observer.getOrderStatus().value() !=
                        vda5050pp::misc::OrderStatus::k_order_active);

                WHEN("The user resumed the order") {
                  auto nav_resumed = std::make_shared<vda5050pp::events::NavigationStatusControl>();
                  nav_resumed->type = vda5050pp::events::NavigationStatusControlType::k_resumed;
                  vda5050pp::core::Instance::ref().getNavigationStatusManager().dispatch(
                      nav_resumed);

                  REQUIRE(resume_result.wait_for(100ms) == std::future_status::ready);

                  THEN("After returning, the scheduler is active") {
                    REQUIRE(order_observer.getOrderStatus().value() ==
                            vda5050pp::misc::OrderStatus::k_order_active);
                  }
                }
              }
            }
          }
        }
      }
    }
    WHEN("A control sink initialtes a pause action non-blockingly") {
      auto pause_result = std::async(std::launch::async, [] {
        vda5050pp::sinks::ControlSink control_sink;
        control_sink.initiatePause("Test caused pause", false);
      });

      THEN("The call returned immediately") {
        REQUIRE(pause_result.wait_for(100ms) == std::future_status::ready);

        THEN("The scheduler is not paused") {
          REQUIRE(order_observer.getOrderStatus().value() !=
                  vda5050pp::misc::OrderStatus::k_order_paused);
        }

        WHEN("The user paused the order") {
          auto nav_paused = std::make_shared<vda5050pp::events::NavigationStatusControl>();
          nav_paused->type = vda5050pp::events::NavigationStatusControlType::k_paused;
          vda5050pp::core::Instance::ref().getNavigationStatusManager().dispatch(nav_paused);

          THEN("The scheduler is paused") {
            REQUIRE(order_observer.getOrderStatus().value() ==
                    vda5050pp::misc::OrderStatus::k_order_paused);
            WHEN("A control sink initialtes a resume action non-blockingly") {
              auto resume_result = std::async(std::launch::async, [] {
                vda5050pp::sinks::ControlSink control_sink;
                control_sink.initiateResume("Test caused resume", false);
              });

              THEN("The call returned immediately") {
                REQUIRE(resume_result.wait_for(100ms) == std::future_status::ready);

                THEN("The scheduler is not resumed") {
                  REQUIRE(order_observer.getOrderStatus().value() !=
                          vda5050pp::misc::OrderStatus::k_order_active);
                }

                WHEN("The user resumed the order") {
                  auto nav_resumed = std::make_shared<vda5050pp::events::NavigationStatusControl>();
                  nav_resumed->type = vda5050pp::events::NavigationStatusControlType::k_resumed;
                  vda5050pp::core::Instance::ref().getNavigationStatusManager().dispatch(
                      nav_resumed);

                  THEN("The scheduler is resumed") {
                    REQUIRE(order_observer.getOrderStatus().value() ==
                            vda5050pp::misc::OrderStatus::k_order_active);
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}
