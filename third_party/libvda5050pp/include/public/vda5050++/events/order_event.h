// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef PUBLIC_VDA5050_2B_2B_EVENTS_ORDER_EVENT_H_
#define PUBLIC_VDA5050_2B_2B_EVENTS_ORDER_EVENT_H_

#include <memory>

#include "vda5050++/events/event_type.h"
#include "vda5050/ActionState.h"
#include "vda5050/Edge.h"
#include "vda5050/Error.h"
#include "vda5050/Node.h"

namespace vda5050pp::events {

enum class OrderEventType {
  k_base_changed,
  k_horizon_changed,
  k_action_states_changed,
  k_errors_changed,
  k_last_node_changed,
};

struct OrderEvent : Event<OrderEventType> {};

struct OrderBaseChanged : public EventId<OrderEvent, OrderEventType::k_base_changed> {
  std::vector<std::shared_ptr<const vda5050::Node>> base_nodes;
  std::vector<std::shared_ptr<const vda5050::Edge>> base_edges;
};

struct OrderHorizonChanged : public EventId<OrderEvent, OrderEventType::k_horizon_changed> {
  std::vector<std::shared_ptr<const vda5050::Node>> horz_nodes;
  std::vector<std::shared_ptr<const vda5050::Edge>> horz_edges;
};

struct OrderActionStatesChanged
    : public EventId<OrderEvent, OrderEventType::k_action_states_changed> {
  std::vector<std::shared_ptr<const vda5050::ActionState>> action_states;
};

struct OrderErrorsChanged : public EventId<OrderEvent, OrderEventType::k_errors_changed> {
  std::vector<vda5050::Error> errors;
};

struct OrderLastNodeChanged : public EventId<OrderEvent, OrderEventType::k_last_node_changed> {
  std::string last_node_id;
  std::optional<decltype(vda5050::Node::sequenceId)> seq_id;
};

}  // namespace vda5050pp::events

#endif  // PUBLIC_VDA5050_2B_2B_EVENTS_ORDER_EVENT_H_
