// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
#include <catch2/catch_all.hpp>

#include "test/data.h"
#include "vda5050++/core/instance.h"
#include "vda5050++/version.h"

using namespace std::chrono_literals;

TEST_CASE("core::validation::ValidationEventHandler::handleValidateInstantActions",
          "[core][validation]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_validation_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_state_event_handler_key);

  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto evt_ia = std::make_shared<vda5050pp::core::events::ValidateInstantActionsEvent>();
  auto instant_actions = std::make_shared<vda5050::InstantActions>();
  instant_actions->actions = {
      test::data::mkAction("a1", "type1", vda5050::BlockingType::NONE),
      test::data::mkAction("a2", "type2", vda5050::BlockingType::NONE),
      test::data::mkAction("a3", "type3", vda5050::BlockingType::NONE),
  };
  evt_ia->instant_actions = instant_actions;

  WHEN("The validate event is sent and no response is received") {
    auto result = evt_ia->getFuture();
    instance->getValidationEventManager().dispatch(evt_ia);

    THEN("The result contains one error for each action") {
      REQUIRE(result.wait_for(1s) == std::future_status::ready);
      REQUIRE(result.get().size() == evt_ia->instant_actions->actions.size());

      THEN("The actions are failed in the state") {
        REQUIRE(instance->getOrderManager().getActionState("a1")->actionStatus ==
                vda5050::ActionStatus::FAILED);
        REQUIRE(instance->getOrderManager().getActionState("a2")->actionStatus ==
                vda5050::ActionStatus::FAILED);
        REQUIRE(instance->getOrderManager().getActionState("a3")->actionStatus ==
                vda5050::ActionStatus::FAILED);
      }
    }
  }

  WHEN("The validate event is sent and a response is received") {
    auto sub = instance->getActionEventManager().getScopedActionEventSubscriber();
    sub.subscribe([](std::shared_ptr<vda5050pp::events::ActionValidate> evt) {
      auto tkn = evt->acquireResultToken();
      vda5050::Error err;
      err.errorType = evt->action->actionType;
      tkn.setValue(std::list{err});
    });

    instance->getValidationEventManager().dispatch(evt_ia);

    THEN("The result contains the respective errors for each action") {
      auto result = evt_ia->getFuture();
      REQUIRE(result.valid());
      REQUIRE(result.wait_for(1s) == std::future_status::ready);
      auto errors = result.get();
      REQUIRE(errors.size() == evt_ia->instant_actions->actions.size());
      std::set<std::string, std::less<>> seen;
      for (const auto &err : errors) {
        seen.insert(err.errorType);
      }
      for (const auto &action : evt_ia->instant_actions->actions) {
        REQUIRE(seen.find(action.actionType) != seen.end());
      }
    }
  }
}

TEST_CASE("core::validation::ValidationEventHandler::handleValidateOrder", "[core][validation]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_validation_event_handler_key);

  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto order = std::make_shared<vda5050::Order>(test::data::mkTemplateOrder({
      test::data::TemplateElement{
          "n0", 0, true, {test::data::mkAction("a1", "type1", vda5050::BlockingType::NONE)}},
      test::data::TemplateElement{
          "e1", 1, true, {test::data::mkAction("a2", "type2", vda5050::BlockingType::NONE)}},
      test::data::TemplateElement{
          "n2", 2, true, {test::data::mkAction("a3", "type3", vda5050::BlockingType::NONE)}},
  }));
  order->zoneSetId = "test_zone_set";
  order->orderId = "test";
  order->orderUpdateId = 0;
  order->header.version = vda5050pp::version::getCurrentVersion();
  auto evt_order = std::make_shared<vda5050pp::core::events::ValidateOrderEvent>();
  evt_order->order = order;

  WHEN("The order is validated without handling actions") {
    auto sub = instance->getQueryEventManager().getScopedQueryEventSubscriber();
    sub.subscribe([](std::shared_ptr<vda5050pp::events::QueryAcceptZoneSet> evt) {
      vda5050::Error err;
      err.errorType = evt->zone_set_id;
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list{err});
    });
    sub.subscribe([](std::shared_ptr<vda5050pp::events::QueryNodeTriviallyReachable> evt) {
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list<vda5050::Error>{});
    });

    auto result = evt_order->getFuture();
    instance->getValidationEventManager().dispatch(evt_order);

    THEN("The result contains one error for each action and a query error") {
      REQUIRE(result.wait_for(1s) == std::future_status::ready);
      auto errors = result.get();
      REQUIRE(errors.size() == 4);
    }
  }

  WHEN("The order is validated with handling actions") {
    auto sub_a = instance->getActionEventManager().getScopedActionEventSubscriber();
    sub_a.subscribe([](std::shared_ptr<vda5050pp::events::ActionValidate> evt) {
      auto tkn = evt->acquireResultToken();
      vda5050::Error err;
      err.errorType = evt->action->actionType;
      tkn.setValue(std::list{err});
    });
    auto sub_q = instance->getQueryEventManager().getScopedQueryEventSubscriber();
    sub_q.subscribe([](std::shared_ptr<vda5050pp::events::QueryAcceptZoneSet> evt) {
      vda5050::Error err;
      err.errorType = evt->zone_set_id;
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list{err});
    });
    sub_q.subscribe([](std::shared_ptr<vda5050pp::events::QueryNodeTriviallyReachable> evt) {
      vda5050::Error err;
      err.errorType = evt->node->nodeId;
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list{err});
    });

    instance->getValidationEventManager().dispatch(evt_order);

    THEN("The result contains the respective errors for each action and the query") {
      auto result = evt_order->getFuture();
      REQUIRE(result.valid());
      REQUIRE(result.wait_for(1s) == std::future_status::ready);
      auto errors = result.get();
      REQUIRE(errors.size() == 5);
      std::set<std::string, std::less<>> seen;
      for (const auto &err : errors) {
        seen.insert(err.errorType);
      }
      for (const auto &action : {"type1", "type2", "type3", "n0"}) {
        REQUIRE(seen.find(action) != seen.end());
      }
    }
  }
}

TEST_CASE("core::validation::ValidationEventHandler::handleValidateOrder - duplicate action ids on "
          "first node",
          "[core][validation]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_message_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_state_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_validation_event_handler_key);
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_interpreter_event_handler_key);

  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  auto initial_order = std::make_shared<vda5050::Order>(test::data::mkTemplateOrder({
      test::data::TemplateElement{
          "n0", 0, true, {test::data::mkAction("a1", "type1", vda5050::BlockingType::NONE)}},
      test::data::TemplateElement{
          "e1", 1, true, {test::data::mkAction("a2", "type2", vda5050::BlockingType::NONE)}},
      test::data::TemplateElement{
          "n2", 2, true, {test::data::mkAction("a3", "type3", vda5050::BlockingType::NONE)}},
  }));
  initial_order->orderId = "test";
  initial_order->orderUpdateId = 0;
  initial_order->header.version = vda5050pp::version::getCurrentVersion();
  auto evt_init_order = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
  evt_init_order->valid_order = initial_order;
  instance->getMessageEventManager().synchronousDispatch(evt_init_order);

  REQUIRE(instance->getOrderManager().getCurrentGraph().bounds().first == 0);
  REQUIRE(instance->getOrderManager().getCurrentGraph().bounds().second == 2);

  auto valid_order = std::make_shared<vda5050::Order>(test::data::mkTemplateOrder({
      test::data::TemplateElement{
          "n2", 2, true, {test::data::mkAction("a3", "type3", vda5050::BlockingType::NONE)}},
      test::data::TemplateElement{"e2", 3, true, {}},
      test::data::TemplateElement{"n3", 4, true, {}},
  }));
  valid_order->orderId = "test";
  valid_order->orderUpdateId = 1;
  valid_order->header.version = vda5050pp::version::getCurrentVersion();
  auto evt_valid_order = std::make_shared<vda5050pp::core::events::ValidateOrderEvent>();
  evt_valid_order->order = valid_order;

  auto invalid_order = std::make_shared<vda5050::Order>(test::data::mkTemplateOrder({
      test::data::TemplateElement{
          "n2", 2, true, {test::data::mkAction("a3", "type4", vda5050::BlockingType::NONE)}},
      test::data::TemplateElement{"e2", 3, true, {}},
      test::data::TemplateElement{"n3", 4, true, {}},
  }));
  invalid_order->orderId = "test";
  invalid_order->orderUpdateId = 1;
  invalid_order->header.version = vda5050pp::version::getCurrentVersion();
  auto evt_invalid_order = std::make_shared<vda5050pp::core::events::ValidateOrderEvent>();
  evt_invalid_order->order = invalid_order;

  WHEN("A valid appending order (with an action at the first node) is dispatched") {
    instance->getValidationEventManager().synchronousDispatch(evt_valid_order);

    THEN("No error is returned") {
      auto future = evt_valid_order->getFuture();
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      auto result = future.get();
      REQUIRE(result.empty());
    }
  }

  WHEN("An invalid appending order (first node action type mismatches) is dispatched") {
    instance->getValidationEventManager().synchronousDispatch(evt_invalid_order);

    THEN("An error is returned") {
      auto future = evt_invalid_order->getFuture();
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      auto result = future.get();
      REQUIRE_FALSE(result.empty());
    }
  }
}