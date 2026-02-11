// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include <catch2/catch_all.hpp>

#include "vda5050++/core/instance.h"

#include "test/data.h"

#include <random>

TEST_CASE("core::messages::MessageEventHandler behaviour", "[core][messages][events]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().useWhiteList();
  cfg.refGlobalConfig().bwListModule(vda5050pp::core::module_keys::k_message_event_handler_key);
  cfg.refGlobalConfig().setLogLevel(vda5050pp::config::LogLevel::k_debug);

  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  WHEN("An order with duplicate order id is received") {
    auto node = std::make_shared<vda5050::Node>();
    node->released = true;
    instance->getOrderManager().replaceGraph(
        vda5050pp::core::state::Graph({vda5050pp::core::state::GraphElement(node)}), "order_id");

    bool valid_called = false;
    auto valid_sub = instance->getValidationEventManager().getScopedSubscriber();
    valid_sub.subscribe<vda5050pp::core::events::ValidateOrderEvent>(
        [&valid_called](auto) { valid_called = true; });

    auto evt = std::make_shared<vda5050pp::core::events::ReceiveOrderMessageEvent>();
    evt->order = std::make_shared<vda5050::Order>();
    evt->order->orderId = "order_id";
    evt->order->orderUpdateId = 0;

    instance->getMessageEventManager().dispatch(evt);

    THEN("No validate oder event was dispatched") { REQUIRE_FALSE(valid_called); }
  }

  WHEN("A new order is received and validation succeeds") {
    bool valid_order_called = false;
    auto msg_sub = instance->getMessageEventManager().getScopedSubscriber();
    msg_sub.subscribe<vda5050pp::core::events::ValidOrderMessageEvent>(
        [&valid_order_called](auto) { valid_order_called = true; });

    auto valid_sub = instance->getValidationEventManager().getScopedSubscriber();
    valid_sub.subscribe<vda5050pp::core::events::ValidateOrderEvent>([](auto evt) {
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list<vda5050::Error>{});
    });

    auto evt = std::make_shared<vda5050pp::core::events::ReceiveOrderMessageEvent>();
    evt->order = std::make_shared<vda5050::Order>();
    evt->order->orderId = "order_id";
    evt->order->orderUpdateId = 0;

    instance->getMessageEventManager().dispatch(evt);

    THEN("ValidOrderMessageEvent was dispatched") { REQUIRE(valid_order_called); }
  }

  WHEN("A new order is received and validation succeeds") {
    bool valid_order_called = false;
    bool order_nodes_sorted = true;
    bool order_edges_sorted = true;
    auto msg_sub = instance->getMessageEventManager().getScopedSubscriber();
    msg_sub.subscribe<vda5050pp::core::events::ValidOrderMessageEvent>(
      [&valid_order_called, &order_nodes_sorted, &order_edges_sorted](auto valid_order_msg_event) {
        valid_order_called = true;
        auto order = valid_order_msg_event->valid_order;

        // Is any node out of order?
        int64_t prev_seq = -1;
        for(auto node : order->nodes) {
          int64_t node_seq_id = (int64_t)node.sequenceId;
          if(prev_seq > node_seq_id) {
            order_nodes_sorted = false;
            break;
          }

          prev_seq = node_seq_id;
        }

        // Is any edge out of order?
        prev_seq = -1;
        for(auto edge : order->edges) {
          int64_t edge_seq_id = (int64_t)edge.sequenceId;
          if(prev_seq > edge_seq_id) {
            order_edges_sorted = false;
            break;
          }

          prev_seq = edge_seq_id;
        }
        
    });

    auto valid_sub = instance->getValidationEventManager().getScopedSubscriber();
    valid_sub.subscribe<vda5050pp::core::events::ValidateOrderEvent>([](auto evt) {
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list<vda5050::Error>{});
    });

    auto evt = std::make_shared<vda5050pp::core::events::ReceiveOrderMessageEvent>();
    evt->order = std::make_shared<vda5050::Order>();
    evt->order->orderId = "order_id";
    evt->order->orderUpdateId = 0;

    instance->getMessageEventManager().dispatch(evt);

    std::vector<test::data::TemplateElement> blueprint;
    blueprint.reserve(20);
    for(uint32_t i = 0; i < 20; i++)
      blueprint.push_back({"test_order", i, true, {}});

    std::mt19937 gen(Catch::getSeed());
    std::shuffle(blueprint.begin(), blueprint.end(), gen);

    evt = std::make_shared<vda5050pp::core::events::ReceiveOrderMessageEvent>();
    evt->order = std::make_shared<vda5050::Order>(test::data::mkTemplateOrder(std::move(blueprint)));
    evt->order->orderId = "test_order";
    evt->order->orderUpdateId = 0;

    instance->getMessageEventManager().dispatch(evt);

    THEN("ValidOrderMessageEvent was dispatched") { REQUIRE(valid_order_called); }
    THEN("Nodes have been sorted") { REQUIRE(order_nodes_sorted); }
    THEN("Edges have been sorted") { REQUIRE(order_edges_sorted); }
  }

  WHEN("A new order is received and validation does not succeed") {
    vda5050::Error e1;
    vda5050::Error e2;
    vda5050::Error e3;
    e1.errorType = "t1";
    e2.errorType = "t2";
    e3.errorType = "t3";

    auto valid_sub = instance->getValidationEventManager().getScopedSubscriber();
    valid_sub.subscribe<vda5050pp::core::events::ValidateOrderEvent>([&e1, &e2, &e3](auto evt) {
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list{e1, e2, e3});
    });

    std::list<vda5050::Error> received_errors;
    auto err_sub = instance->getStatusEventManager().getScopedStatusEventSubscriber();
    err_sub.subscribe([&received_errors](std::shared_ptr<vda5050pp::events::ErrorAdd> evt) {
      received_errors.push_back(evt->error);
    });

    auto evt = std::make_shared<vda5050pp::core::events::ReceiveOrderMessageEvent>();
    evt->order = std::make_shared<vda5050::Order>();
    evt->order->orderId = "order_id";
    evt->order->orderUpdateId = 0;

    instance->getMessageEventManager().dispatch(evt);

    THEN("All errors were dispatched") { REQUIRE(received_errors == std::list{e1, e2, e3}); }
  }

  WHEN("Instant Actions are received and validation succeeds") {
    bool valid_ia_called = false;
    auto msg_sub = instance->getMessageEventManager().getScopedSubscriber();
    msg_sub.subscribe<vda5050pp::core::events::ValidInstantActionMessageEvent>(
        [&valid_ia_called](auto) { valid_ia_called = true; });

    auto valid_sub = instance->getValidationEventManager().getScopedSubscriber();
    valid_sub.subscribe<vda5050pp::core::events::ValidateInstantActionsEvent>([](auto evt) {
      auto tkn = evt->acquireResultToken();
      tkn.setValue(std::list<vda5050::Error>{});
    });

    auto evt = std::make_shared<vda5050pp::core::events::ReceiveInstantActionMessageEvent>();
    evt->instant_actions = std::make_shared<vda5050::InstantActions>();

    instance->getMessageEventManager().dispatch(evt);

    THEN("ValidInstantActionMessageEvent was dispatched") { REQUIRE(valid_ia_called); }
  }

  WHEN("InstantActions are received and validation does not succeed") {
    vda5050::Error e1;
    vda5050::Error e2;
    vda5050::Error e3;
    e1.errorType = "t1";
    e2.errorType = "t2";
    e3.errorType = "t3";

    auto valid_sub = instance->getValidationEventManager().getScopedSubscriber();
    valid_sub.subscribe<vda5050pp::core::events::ValidateInstantActionsEvent>(
        [&e1, &e2, &e3](auto evt) {
          auto tkn = evt->acquireResultToken();
          tkn.setValue(std::list{e1, e2, e3});
        });

    std::list<vda5050::Error> received_errors;
    auto err_sub = instance->getStatusEventManager().getScopedStatusEventSubscriber();
    err_sub.subscribe([&received_errors](std::shared_ptr<vda5050pp::events::ErrorAdd> evt) {
      received_errors.push_back(evt->error);
    });

    auto evt = std::make_shared<vda5050pp::core::events::ReceiveInstantActionMessageEvent>();
    evt->instant_actions = std::make_shared<vda5050::InstantActions>();

    instance->getMessageEventManager().dispatch(evt);

    THEN("All errors were dispatched") { REQUIRE(received_errors == std::list{e1, e2, e3}); }
  }
}