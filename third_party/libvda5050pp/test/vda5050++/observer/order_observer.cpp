// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include <vda5050++/core/instance.h>
#include <vda5050++/observer/order_observer.h>

#include <catch2/catch_all.hpp>
#include <chrono>

#include "test/data.h"
using namespace std::chrono_literals;

TEST_CASE("observer::OrderObserver - event handling", "[observer]") {
  vda5050pp::core::events::EventManagerOptions evt_opts;
  evt_opts.synchronous_event_dispatch = true;
  vda5050pp::Config cfg;
  cfg.refGlobalConfig().useWhiteList();
  vda5050pp::core::Instance::reset();
  auto instance = vda5050pp::core::Instance::init(cfg, evt_opts).lock();

  vda5050pp::observer::OrderObserver observer;

  WHEN("No events were received yet") {
    THEN("The last node ID is (\"\", 0)") { REQUIRE_FALSE(observer.getLastNode().has_value()); }

    THEN("The order status is k_order_idle") {
      REQUIRE_FALSE(observer.getOrderStatus().has_value());
    }

    THEN("The actions are not known") {
      REQUIRE_FALSE(observer.getActionStatus("action1").has_value());
      REQUIRE_FALSE(observer.getActionStatus("action2").has_value());
      REQUIRE_FALSE(observer.getActionStatus("").has_value());
    }
  }

  WHEN("Receiving a NewLastNode Event") {
    auto evt = std::make_shared<vda5050pp::core::events::OrderNewLastNodeId>();
    evt->last_node_id = "node1";
    evt->seq_id = 1;

    std::string cb_id;
    decltype(vda5050::Node::sequenceId) cb_seq = 0;

    observer.onLastNodeIdChanged([&cb_id, &cb_seq](auto pair) {
      const auto &[id, seq] = pair;
      cb_id = id;
      cb_seq = seq.value_or(0);
    });

    instance->getOrderEventManager().dispatch(evt);

    THEN("The last node ID is (\"node1\", 1)") {
      auto ret = observer.getLastNode();
      REQUIRE(ret.has_value());
      const auto &[id, seq] = ret.value();
      REQUIRE(id == "node1");
      REQUIRE(seq == 1);
    }

    THEN("The callback was called") {
      REQUIRE(cb_id == "node1");
      REQUIRE(cb_seq == 1);
    }
  }

  WHEN("Receiving an OrderStatus Event") {
    auto evt = std::make_shared<vda5050pp::core::events::OrderStatus>();
    evt->status = vda5050pp::misc::OrderStatus::k_order_active;

    vda5050pp::misc::OrderStatus cb_status = vda5050pp::misc::OrderStatus::k_order_idle;

    observer.onOrderStatusChanged([&cb_status](auto status) { cb_status = status; });

    instance->getOrderEventManager().dispatch(evt);

    THEN("The order status is correct") {
      REQUIRE(observer.getOrderStatus() == vda5050pp::misc::OrderStatus::k_order_active);
    }

    THEN("The callback was called") {
      REQUIRE(cb_status == vda5050pp::misc::OrderStatus::k_order_active);
    }
  }

  WHEN("Receiving a ValidOrderMessageEvent") {
    auto evt = std::make_shared<vda5050pp::core::events::ValidOrderMessageEvent>();
    evt->valid_order = std::make_shared<vda5050::Order>();
    evt->valid_order->orderId = "order1";
    evt->valid_order->orderUpdateId = 1;

    std::promise<std::pair<std::string, uint32_t>> promise;
    auto future = promise.get_future();

    observer.onOrderIdChanged([&promise](const auto &id) { promise.set_value(id); });

    instance->getMessageEventManager().dispatch(evt);

    THEN("The order ID is correct") {
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      auto [id, seq] = future.get();
      REQUIRE(id == "order1");
      REQUIRE(seq == 1);
    }

    THEN("The getter returns the correct values") {
      auto ret = observer.getOrderId();

      REQUIRE(ret.has_value());
      const auto &[id, seq] = ret.value();
      REQUIRE(id == "order1");
      REQUIRE(seq == 1);
    }
  }

  WHEN("Receiving an ActionStatus Event") {
    auto evt = std::make_shared<vda5050pp::core::events::OrderActionStatusChanged>();
    evt->action_id = "action1";
    evt->action_status = vda5050::ActionStatus::RUNNING;

    auto evt2 = std::make_shared<vda5050pp::events::OrderActionStatesChanged>();
    evt2->action_states = {test::data::wrap_shared(vda5050::ActionState{
        "action1",
        std::nullopt,
        std::nullopt,
        vda5050::ActionStatus::WAITING,
        std::nullopt,
    })};

    vda5050::ActionStatus cb_status = vda5050::ActionStatus::WAITING;
    bool cb2_called = false;
    std::vector<std::shared_ptr<const vda5050::ActionState>> action_states;

    observer.onActionStatusChanged("action1", [&cb_status](auto status) { cb_status = status; });
    observer.onActionStatusChanged("action2", [&cb2_called](auto) { cb2_called = true; });
    observer.onActionStatusChanged([&action_states](const auto &as) { action_states = as; });

    instance->getOrderEventManager().dispatch(evt);
    instance->getAgvOrderEventManager().dispatch(evt2);

    THEN("The action status is correct") {
      REQUIRE(observer.getActionStatus("action1") == vda5050::ActionStatus::RUNNING);
    }

    THEN("The callback was called") { REQUIRE(cb_status == vda5050::ActionStatus::RUNNING); }

    THEN("Then other action callbacks were not called") { REQUIRE_FALSE(cb2_called); }

    THEN("All current action states were dispatched") {
      REQUIRE(action_states.size() == 1);
      REQUIRE(action_states[0]->actionId == "action1");
    }
  }

  WHEN("Receiving a BaseChanged Event") {
    auto evt = std::make_shared<vda5050pp::events::OrderBaseChanged>();
    evt->base_nodes = {
        test::data::wrap_shared(test::data::mkNode("n1", 0, true, {})),
        test::data::wrap_shared(test::data::mkNode("n2", 2, true, {})),
    };
    evt->base_edges = {
        test::data::wrap_shared(test::data::mkEdge("e1", 1, true, {})),
    };

    std::promise<std::pair<std::vector<std::shared_ptr<const vda5050::Node>>,
                           std::vector<std::shared_ptr<const vda5050::Edge>>>>
        promise;
    auto future = promise.get_future();

    observer.onBaseChanged([&promise](const auto &nodes, const auto &edges) {
      promise.set_value(std::make_pair(nodes, edges));
    });

    vda5050pp::core::Instance::ref().getAgvOrderEventManager().dispatch(evt);

    THEN("It contains the correct base") {
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      auto [nodes, edges] = future.get();
      REQUIRE(nodes.size() == 2);
      REQUIRE(edges.size() == 1);
      REQUIRE(nodes[0]->nodeId == "n1");
      REQUIRE(nodes[1]->nodeId == "n2");
      REQUIRE(edges[0]->edgeId == "e1");
    }
  }

  WHEN("Receiving a HorizonChanged Event") {
    auto evt = std::make_shared<vda5050pp::events::OrderHorizonChanged>();
    evt->horz_nodes = {
        test::data::wrap_shared(test::data::mkNode("n1", 0, false, {})),
        test::data::wrap_shared(test::data::mkNode("n2", 2, false, {})),
    };
    evt->horz_edges = {
        test::data::wrap_shared(test::data::mkEdge("e1", 1, false, {})),
    };

    std::promise<std::pair<std::vector<std::shared_ptr<const vda5050::Node>>,
                           std::vector<std::shared_ptr<const vda5050::Edge>>>>
        promise;
    auto future = promise.get_future();

    observer.onHorizonChanged([&promise](const auto &nodes, const auto &edges) {
      promise.set_value(std::make_pair(nodes, edges));
    });

    vda5050pp::core::Instance::ref().getAgvOrderEventManager().dispatch(evt);

    THEN("It contains the correct horizon") {
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      auto [nodes, edges] = future.get();
      REQUIRE(nodes.size() == 2);
      REQUIRE(edges.size() == 1);
      REQUIRE(nodes[0]->nodeId == "n1");
      REQUIRE(nodes[1]->nodeId == "n2");
      REQUIRE(edges[0]->edgeId == "e1");
    }
  }

  WHEN("Receiving a ErrorsChanged Event") {
    auto evt = std::make_shared<vda5050pp::events::OrderErrorsChanged>();
    evt->errors = {
        vda5050::Error{"e1", std::nullopt, std::nullopt, std::nullopt,
                       vda5050::ErrorLevel::WARNING},
        vda5050::Error{"e2", std::nullopt, std::nullopt, std::nullopt,
                       vda5050::ErrorLevel::WARNING},
    };

    std::promise<std::vector<vda5050::Error>> promise;
    auto future = promise.get_future();

    observer.onErrorsChanged([&promise](const auto &errors) { promise.set_value(errors); });

    vda5050pp::core::Instance::ref().getAgvOrderEventManager().dispatch(evt);

    THEN("It contains the correct errors") {
      REQUIRE(future.wait_for(100ms) == std::future_status::ready);
      auto errors = future.get();
      REQUIRE(errors.size() == 2);
      REQUIRE(errors[0].errorType == "e1");
      REQUIRE(errors[1].errorType == "e2");
    }
  }
}