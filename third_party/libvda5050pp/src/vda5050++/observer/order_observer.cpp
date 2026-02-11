// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#include "vda5050++/observer/order_observer.h"

#include "vda5050++/core/common/exception.h"
#include "vda5050++/core/events/message_event.h"
#include "vda5050++/core/events/order_event.h"
#include "vda5050++/core/instance.h"

using namespace vda5050pp::observer;

///
///\brief This class contains private dependencies of the library and must be stored with type
/// vanishing.
/// Otherwise this would cludder the public interface of the library.
///
struct OpaqueOrderSubscriber {
  vda5050pp::core::GenericEventManager<vda5050pp::core::events::OrderEvent>::ScopedSubscriber
      order_event_subscriber;
};

///
///\brief This class contains private dependencies of the library and must be stored with type
/// vanishing.
/// Otherwise this would cludder the public interface of the library.
/// NOTE: This split is somehow required to prevent memory corruption. This needs further
/// investigation.
///
struct OpaqueMessageSubscriber {
  vda5050pp::core::GenericEventManager<vda5050pp::core::events::MessageEvent>::ScopedSubscriber
      message_event_subscriber;
};

///
///\brief This class contains private dependencies of the library and must be stored with type
/// vanishing.
/// Otherwise this would cludder the public interface of the library.
/// NOTE: This split is somehow required to prevent memory corruption. This needs further
/// investigation.
///
struct OpaqueNavigationStatusSubscriber {
  vda5050pp::core::ScopedNavigationStatusSubscriber navigation_status_subscriber;
};

inline OpaqueOrderSubscriber &getOpaqueOrderSubscriber(vda5050pp::misc::AnyPtr &ptr) {
  return *ptr.get<OpaqueOrderSubscriber>();
}

inline OpaqueMessageSubscriber &getOpaqueMessageSubscriber(vda5050pp::misc::AnyPtr &ptr) {
  return *ptr.get<OpaqueMessageSubscriber>();
}

inline OpaqueNavigationStatusSubscriber &getOpaqueNavigationStatusSubscriber(
    vda5050pp::misc::AnyPtr &ptr) {
  return *ptr.get<OpaqueNavigationStatusSubscriber>();
}

OrderObserver::OrderObserver() {
  auto opaque_order_subscriber = std::make_shared<OpaqueOrderSubscriber>(OpaqueOrderSubscriber{
      vda5050pp::core::Instance::ref().getOrderEventManager().getScopedSubscriber()});
  auto opaque_message_subscriber =
      std::make_shared<OpaqueMessageSubscriber>(OpaqueMessageSubscriber{
          vda5050pp::core::Instance::ref().getMessageEventManager().getScopedSubscriber()});
  auto opaque_navigation_status_subscriber = std::make_shared<OpaqueNavigationStatusSubscriber>(
      OpaqueNavigationStatusSubscriber{vda5050pp::core::Instance::ref()
                                           .getNavigationStatusManager()
                                           .getScopedNavigationStatusSubscriber()});

  opaque_order_subscriber->order_event_subscriber
      .subscribe<vda5050pp::core::events::OrderNewLastNodeId>(
          [this](std::shared_ptr<vda5050pp::core::events::OrderNewLastNodeId> event) {
            std::unique_lock lock(this->last_node_mutex_);
            this->last_node_id_ = event->last_node_id;
            this->last_seq_id_ = event->seq_id;
          });
  opaque_order_subscriber->order_event_subscriber
      .subscribe<vda5050pp::core::events::OrderActionStatusChanged>(
          [this](std::shared_ptr<vda5050pp::core::events::OrderActionStatusChanged> event) {
            std::unique_lock lock(this->action_status_mutex_);
            this->action_status_[event->action_id] = event->action_status;
          });
  opaque_order_subscriber->order_event_subscriber.subscribe<vda5050pp::core::events::OrderStatus>(
      [this](std::shared_ptr<vda5050pp::core::events::OrderStatus> event) {
        std::unique_lock lock(this->order_status_mutex_);
        this->order_status_ = event->status;
      });
  opaque_message_subscriber->message_event_subscriber
      .subscribe<vda5050pp::core::events::ValidOrderMessageEvent>(
          [this](std::shared_ptr<vda5050pp::core::events::ValidOrderMessageEvent> event) {
            std::unique_lock lock(this->order_id_mutex_);
            this->order_id_ = {event->valid_order->orderId, event->valid_order->orderUpdateId};
          });

  this->opaque_order_subscriber_ = opaque_order_subscriber;
  this->opaque_message_subscriber_ = opaque_message_subscriber;
  this->opaque_navigation_status_subscriber_ = opaque_navigation_status_subscriber;
  this->order_event_subscriber_ =
      vda5050pp::core::Instance::ref().getAgvOrderEventManager().getScopedOrderEventSubscriber();
}

std::optional<std::pair<std::string, decltype(vda5050::Node::sequenceId)>>
OrderObserver::getLastNode() const {
  std::shared_lock lock(this->last_node_mutex_);

  if (!this->last_node_id_ || !this->last_seq_id_) {
    return std::nullopt;
  }

  return {{this->last_node_id_.value(), this->last_seq_id_.value()}};
}

std::optional<vda5050::ActionStatus> OrderObserver::getActionStatus(
    std::string_view action_id) const {
  std::shared_lock lock(this->action_status_mutex_);

  if (auto it = this->action_status_.find(action_id); it != this->action_status_.end()) {
    return it->second;
  } else {
    return std::nullopt;
  }
}

std::optional<vda5050pp::misc::OrderStatus> OrderObserver::getOrderStatus() const {
  std::shared_lock lock(this->order_status_mutex_);
  return this->order_status_;
}

std::optional<std::pair<std::string, uint32_t>> OrderObserver::getOrderId() const {
  std::shared_lock lock(this->order_id_mutex_);
  return this->order_id_;
}

void OrderObserver::onOrderStatusChanged(
    std::function<void(vda5050pp::misc::OrderStatus)> &&callback) {
  getOpaqueOrderSubscriber(this->opaque_order_subscriber_)
      .order_event_subscriber.subscribe<vda5050pp::core::events::OrderStatus>(
          [cb = std::move(callback)](std::shared_ptr<vda5050pp::core::events::OrderStatus> event) {
            cb(event->status);
          });
}

void OrderObserver::onOrderIdChanged(
    std::function<void(std::pair<std::string, uint32_t>)> &&callback) {
  getOpaqueMessageSubscriber(this->opaque_message_subscriber_)
      .message_event_subscriber.subscribe<vda5050pp::core::events::ValidOrderMessageEvent>(
          [cb = std::move(callback)](
              std::shared_ptr<vda5050pp::core::events::ValidOrderMessageEvent> event) {
            cb({event->valid_order->orderId, event->valid_order->orderUpdateId});
          });
}

void OrderObserver::onActionStatusChanged(std::string_view action_id,
                                          std::function<void(vda5050::ActionStatus)> &&callback) {
  getOpaqueOrderSubscriber(this->opaque_order_subscriber_)
      .order_event_subscriber.subscribe<vda5050pp::core::events::OrderActionStatusChanged>(
          [a_id = std::string(action_id), cb = std::move(callback)](
              std::shared_ptr<vda5050pp::core::events::OrderActionStatusChanged> event) {
            if (a_id == event->action_id) {
              cb(event->action_status);
            }
          });
}

void OrderObserver::onActionStatusChanged(
    std::function<void(const std::vector<std::shared_ptr<const vda5050::ActionState>> &)>
        &&callback) const {
  this->order_event_subscriber_->subscribe(
      [cb = std::move(callback)](
          std::shared_ptr<vda5050pp::events::OrderActionStatesChanged> event) {
        cb(event->action_states);
      });
}

void OrderObserver::onLastNodeIdChanged(
    std::function<void(std::pair<std::string_view,
                                 std::optional<decltype(vda5050::Node::sequenceId)>>)> &&callback) {
  getOpaqueOrderSubscriber(this->opaque_order_subscriber_)
      .order_event_subscriber.subscribe<vda5050pp::core::events::OrderNewLastNodeId>(
          [cb = callback](std::shared_ptr<vda5050pp::core::events::OrderNewLastNodeId> event) {
            cb({event->last_node_id, event->seq_id});
          });

  getOpaqueNavigationStatusSubscriber(this->opaque_navigation_status_subscriber_)
      .navigation_status_subscriber.subscribe(
          [cb = std::move(callback)](
              std::shared_ptr<vda5050pp::events::NavigationStatusNodeReached> event) {
            if (event->last_node_id.has_value()) {
              cb({*event->last_node_id, event->node_seq_id});
            }
          });
}

void OrderObserver::onBaseChanged(
    std::function<void(const std::vector<std::shared_ptr<const vda5050::Node>> &,
                       const std::vector<std::shared_ptr<const vda5050::Edge>> &)> &&callback)
    const {
  this->order_event_subscriber_->subscribe(
      [cb = std::move(callback)](std::shared_ptr<vda5050pp::events::OrderBaseChanged> event) {
        cb(event->base_nodes, event->base_edges);
      });
}

void OrderObserver::onHorizonChanged(
    std::function<void(const std::vector<std::shared_ptr<const vda5050::Node>> &,
                       const std::vector<std::shared_ptr<const vda5050::Edge>> &)> &&callback)
    const {
  this->order_event_subscriber_->subscribe(
      [cb = std::move(callback)](std::shared_ptr<vda5050pp::events::OrderHorizonChanged> event) {
        cb(event->horz_nodes, event->horz_edges);
      });
}

void OrderObserver::onErrorsChanged(
    std::function<void(const std::vector<vda5050::Error> &)> &&callback) const {
  this->order_event_subscriber_->subscribe(
      [cb = std::move(callback)](std::shared_ptr<vda5050pp::events::OrderErrorsChanged> event) {
        cb(event->errors);
      });
}
