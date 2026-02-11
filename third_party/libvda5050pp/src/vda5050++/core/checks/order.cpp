//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/core/checks/order.h"

#include <set>

#include "vda5050++/core/checks/action.h"
#include "vda5050++/core/common/exception.h"
#include "vda5050++/core/instance.h"

std::list<vda5050::Error> vda5050pp::core::checks::checkOrderGraphConsistency(
    const vda5050::Order &order) {
  (void)order;
  std::set<uint32_t> found_base;
  std::set<uint32_t> found_horz;

  if (order.nodes.empty()) {
    return {{"orderError",
             {{{"order.orderId", order.orderId},
               {"order.orderUpdateId", std::to_string(order.orderUpdateId)}}},
             "This order does not contain any nodes",
             "Do not send empty orders, if you intend to cancel this order, please use the "
             "cancelOrder instant action",
             vda5050::ErrorLevel::WARNING}};
  }

  // Check node sequence ids
  for (auto &node : order.nodes) {
    if (node.sequenceId % 2 == 1) {
      return {{"orderError",
               {{{"order.orderId", order.orderId},
                 {"order.orderUpdateId", std::to_string(order.orderUpdateId)},
                 {"node.nodeId", node.nodeId},
                 {"node.sequenceId", std::to_string(node.sequenceId)}}},
               "The order contains a node with an odd sequence id",
               "Make sure all Nodes have even sequence ids and all Edges have odd sequence ids",
               vda5050::ErrorLevel::WARNING}};
    }
    if (found_base.find(node.sequenceId) != found_base.end() ||
        found_horz.find(node.sequenceId) != found_horz.end()) {
      return {{"orderError",
               {{{"order.orderId", order.orderId},
                 {"order.orderUpdateId", std::to_string(order.orderUpdateId)},
                 {"node.nodeId", node.nodeId},
                 {"node.sequenceId", std::to_string(node.sequenceId)}}},
               "The order contains duplicate sequence ids",
               "Make sure all Nodes have unique sequence ids",
               vda5050::ErrorLevel::WARNING}};
    }
    if (node.released) {
      found_base.insert(node.sequenceId);
    } else {
      found_horz.insert(node.sequenceId);
    }
  }

  // Check edge sequence ids
  for (auto &edge : order.edges) {
    if (edge.sequenceId % 2 == 0) {
      return {{"orderError",
               {{{"order.orderId", order.orderId},
                 {"order.orderUpdateId", std::to_string(order.orderUpdateId)},
                 {"edge.edgeId", edge.edgeId},
                 {"edge.sequenceId", std::to_string(edge.sequenceId)}}},
               "The order contains a edge with an even sequence id",
               "Make sure all Nodes have even sequence ids and all Edges have odd sequence ids",
               vda5050::ErrorLevel::WARNING}};
    }
    if (found_base.find(edge.sequenceId) != found_base.end() ||
        found_horz.find(edge.sequenceId) != found_horz.end()) {
      return {{"orderError",
               {{{"order.orderId", order.orderId},
                 {"order.orderUpdateId", std::to_string(order.orderUpdateId)},
                 {"edge.edgeId", edge.edgeId},
                 {"edge.sequenceId", std::to_string(edge.sequenceId)}}},
               "The order contains duplicate sequence ids",
               "Make sure all Edges have unique sequence ids",
               vda5050::ErrorLevel::WARNING}};
    }
    if (edge.released) {
      found_base.insert(edge.sequenceId);
    } else {
      found_horz.insert(edge.sequenceId);
    }
  }

  // Check if there are no seqId skips
  uint32_t max_seq = 0;
  uint32_t min_seq = 0;

  if (found_base.empty()) {
    max_seq = *found_horz.crbegin();
    min_seq = *found_horz.cbegin();
  } else if (found_horz.empty()) {
    max_seq = *found_base.crbegin();
    min_seq = *found_base.cbegin();
  } else {
    max_seq = std::max(*found_base.crbegin(), *found_horz.crbegin());
    min_seq = std::min(*found_base.cbegin(), *found_horz.cbegin());
  }

  if (max_seq - min_seq + 1 != found_base.size() + found_horz.size()) {
    return {{"orderError",
             {{{"order.orderId", order.orderId},
               {"order.orderUpdateId", std::to_string(order.orderUpdateId)}}},
             "The order skips sequence ids",
             "Make sure your order contains a continuous sequence of ids",
             vda5050::ErrorLevel::WARNING}};
  }

  // Check if horizon && base are clearly separated
  if (!found_horz.empty() && !found_base.empty()) {
    auto max_base_seq = *found_base.crbegin();
    auto min_horz_seq = *found_horz.cbegin();
    if (min_horz_seq <= max_base_seq) {
      return {{"orderError",
               {{{"order.orderId", order.orderId},
                 {"order.orderUpdateId", std::to_string(order.orderUpdateId)}}},
               "The order contains a horizon sequence id smaller then a base sequence id",
               "Make sure your order contains a clear separation between base and horizon",
               vda5050::ErrorLevel::WARNING}};
    }
  }

  return {};
}

std::list<vda5050::Error> vda5050pp::core::checks::checkOrderId(const vda5050::Order &order) {
  auto [id, update_id] = Instance::ref().getOrderManager().getOrderId();

  // Do not allow orderUpdateId decrease
  if (id == order.orderId && order.orderUpdateId < update_id) {
    vda5050::Error error;
    error.errorType = "orderUpdateError";
    error.errorDescription = "6.6.4.3 - Cannot accept decreasing orderUpdateId";
    error.errorLevel = vda5050::ErrorLevel::WARNING;
    error.errorHint =
        "Make sure your orderUpdateIds do not increase. Otherwise use another orderId";
    error.errorReferences = {
        {"state.orderId", id},
        {"state.orderUpdateId", std::to_string(update_id)},
        {"order.orderId", order.orderId},
        {"order.orderUpdateId", std::to_string(order.orderUpdateId)},
    };

    return {error};
  }

  // Do not allow orderUpdateId > 0 for new orders
  if (id != order.orderId && order.orderUpdateId > 0) {
    vda5050::Error error;
    error.errorType = "orderUpdateError";
    error.errorDescription = "Cannot accept orderUpdateId > 0 for new order";
    error.errorLevel = vda5050::ErrorLevel::WARNING;
    error.errorReferences = {
        {"state.orderId", id},
        {"state.orderUpdateId", std::to_string(update_id)},
        {"order.orderId", order.orderId},
        {"order.orderUpdateId", std::to_string(order.orderUpdateId)},
    };

    return {error};
  }

  return {};
}

std::list<vda5050::Error> vda5050pp::core::checks::checkOrderAppend(const vda5050::Order &order) {
  auto [id, update_id] = Instance::ref().getOrderManager().getOrderId();

  // Do not validate non-appending orders
  if (order.orderId == id && order.orderUpdateId <= update_id) {
    return {};
  }

  // The Order may replace
  bool may_replace =
      (Instance::ref().getOrderManager().getOrderStatus() ==
           vda5050pp::misc::OrderStatus::k_order_idle ||
       Instance::ref().getOrderManager().getOrderStatus() ==
           vda5050pp::misc::OrderStatus::k_order_idle_paused)  // Scheduler idle
      && (!Instance::ref().getOrderManager().hasGraph() ||
          !Instance::ref().getOrderManager().getCurrentGraph().hasOrder());  // state has no order;

  if (!may_replace && order.orderId != id) {
    return {{"orderUpdateError",
             {{{"order.orderId", order.orderId}, {"state.orderId", id}}},
             "Order ID does not match the current (unfinished) one",
             "Your order replaces the current one, but the current order is not finished yet. Try "
             "to send an order update instead or cancel the current order.",
             vda5050::ErrorLevel::WARNING}};
  }

  // This case will not be checked here
  if (order.nodes.empty()) {
    return {};
  }

  auto cmp_seq = [](auto &n1, auto &n2) { return n1.sequenceId < n2.sequenceId; };
  auto order_first_node_it = std::min_element(cbegin(order.nodes), cend(order.nodes), cmp_seq);
  auto min_seq = order_first_node_it->sequenceId;

  std::optional<uint32_t> base_last = std::nullopt;
  if (Instance::ref().getOrderManager().hasGraph()) {
    auto [_, l] = Instance::ref().getOrderManager().getCurrentGraph().baseBounds();
    base_last = l;
  }

  bool appends = base_last && *base_last == min_seq &&
                 min_seq > 0;  // min_seq == 0 means, that the order replaces the current one

  // Only allow direct appending
  if (auto ok = appends || (may_replace && min_seq == 0); !ok) {
    return {{"orderUpdateError",
             {{{"order.orderId", order.orderId},
               {"order.orderUpdateId", std::to_string(order.orderUpdateId)},
               {"order.node.sequenceId", std::to_string(order_first_node_it->sequenceId)},
               {"state.baseSequenceId", base_last ? std::to_string(*base_last) : "<none>"}}},
             "Could not stitch order due to invalid sequence ids",
             "When appending an order, the first node must have a sequence id equal to the last "
             "base node's sequence id",
             vda5050::ErrorLevel::WARNING}};
  }

  // If it appends, the first nodes must match
  if (appends) {
    const auto &graph = Instance::ref().getOrderManager().getCurrentGraph();
    auto last_base_node = graph.at(graph.baseBounds().second);
    if (last_base_node.getNode()->operator!=(*order_first_node_it)) {
      return {{"orderUpdateError",
               {{{"order.nodes[0]", vda5050::json(*order_first_node_it).dump(2)},
                 {"state.baseNodes[-1]", vda5050::json(*last_base_node.getNode()).dump(2)}}},
               "The first node of the order does not equal the last base node",
               "Make sure your appending order has exactly the same first node as the last node of "
               "the base.",
               vda5050::ErrorLevel::WARNING}};
    }
  }

  return {};
}

std::list<vda5050::Error> vda5050pp::core::checks::checkOrderActionIds(
    const vda5050::Order &order) {
  std::set<std::string_view, std::less<>> seen_ids;
  std::list<vda5050::Error> errors;

  bool skip_first_node = order.orderUpdateId > 0;

  for (const auto &node : order.nodes) {
    if (skip_first_node) {
      skip_first_node = false;
      continue;
    }
    for (const auto &action : node.actions) {
      errors.splice(errors.end(),
                    uniqueActionId(action, seen_ids, vda5050pp::misc::ActionContext::k_node));
    }
  }
  for (const auto &edge : order.edges) {
    for (const auto &action : edge.actions) {
      errors.splice(errors.end(),
                    uniqueActionId(action, seen_ids, vda5050pp::misc::ActionContext::k_edge));
    }
  }

  return errors;
}
