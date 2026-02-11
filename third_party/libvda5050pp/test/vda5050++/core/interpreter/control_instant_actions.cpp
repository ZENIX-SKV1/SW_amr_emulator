// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/core/interpreter/control_instant_actions.h"

#include <catch2/catch_all.hpp>

#include "vda5050++/core/instance.h"

using namespace std::chrono_literals;

TEST_CASE("core::interpreter::makeCancelControlBlock behaviour", "[core][interpreter]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = false;
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().useWhiteList();
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto action = std::make_shared<vda5050::Action>();
  action->actionType = "cancelOrder";
  action->actionId = "test";

  auto control_block = vda5050pp::core::interpreter::makeCancelControlBlock(action);

  REQUIRE(control_block != nullptr);

  WHEN("The control block is enabled") {
    auto interpreter_sub = instance->getInterpreterEventManager().getScopedSubscriber();
    auto action_sub = instance->getOrderEventManager().getScopedSubscriber();
    std::promise<vda5050pp::core::events::InterpreterOrderControl::Status> order_control_promise;
    interpreter_sub.subscribe<vda5050pp::core::events::InterpreterOrderControl>(
        [&order_control_promise](auto evt) { order_control_promise.set_value(evt->status); });

    std::promise<vda5050pp::core::events::OrderActionStatusChanged> action_status_changed_promise;
    action_sub.subscribe<vda5050pp::core::events::OrderActionStatusChanged>(
        [&action_status_changed_promise, called = false](auto evt) mutable {
          if (!called) {
            action_status_changed_promise.set_value(*evt);
            called = true;
          }
        });

    control_block->enable();

    THEN("The Interpreter is requested to cancel") {
      auto order_control_future = order_control_promise.get_future();
      REQUIRE(order_control_future.wait_for(100ms) == std::future_status::ready);
      REQUIRE(order_control_future.get() ==
              vda5050pp::core::events::InterpreterOrderControl::Status::k_cancel);
    }
    THEN("The cancel order is running") {
      auto action_status_changed_future = action_status_changed_promise.get_future();
      REQUIRE(action_status_changed_future.wait_for(100ms) == std::future_status::ready);
      REQUIRE(action_status_changed_future.get().action_status == vda5050::ActionStatus::RUNNING);
    }

    WHEN("The interpreter is idle") {
      std::promise<vda5050pp::core::events::OrderActionStatusChanged>
          action_status_changed_promise_2;
      action_sub.subscribe<vda5050pp::core::events::OrderActionStatusChanged>(
          [&action_status_changed_promise_2, called = false](auto evt) mutable {
            if (!called) {
              action_status_changed_promise_2.set_value(*evt);
              called = true;
            }
          });

      auto evt = std::make_shared<vda5050pp::core::events::OrderStatus>();
      evt->status = vda5050pp::misc::OrderStatus::k_order_idle;
      instance->getOrderEventManager().dispatch(evt);

      THEN("The cancel order is finished") {
        auto action_status_changed_future_2 = action_status_changed_promise_2.get_future();
        REQUIRE(action_status_changed_future_2.wait_for(100ms) == std::future_status::ready);
        REQUIRE(action_status_changed_future_2.get().action_status ==
                vda5050::ActionStatus::FINISHED);
      }
    }
  }
}