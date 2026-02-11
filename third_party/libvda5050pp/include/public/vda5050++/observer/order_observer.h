// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef PUBLIC_VDA5050_2B_2B_OBSERVER_ORDER_OBSERVER_H_
#define PUBLIC_VDA5050_2B_2B_OBSERVER_ORDER_OBSERVER_H_

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <utility>

#include "vda5050++/events/scoped_order_event_subscriber.h"
#include "vda5050++/misc/any_ptr.h"
#include "vda5050++/misc/order_status.h"
#include "vda5050/ActionState.h"
#include "vda5050/Edge.h"
#include "vda5050/Error.h"
#include "vda5050/Node.h"

namespace vda5050pp::observer {

///
///\brief The OrderObserver class is used observe the order status inside the library.
///
class OrderObserver {
private:
  mutable std::shared_mutex last_node_mutex_;
  std::optional<std::string> last_node_id_;
  std::optional<decltype(vda5050::Node::sequenceId)> last_seq_id_;

  mutable std::shared_mutex action_status_mutex_;
  std::map<std::string, vda5050::ActionStatus, std::less<>> action_status_;

  mutable std::shared_mutex order_status_mutex_;
  std::optional<vda5050pp::misc::OrderStatus> order_status_;

  mutable std::shared_mutex order_id_mutex_;
  std::optional<std::pair<std::string, uint32_t>> order_id_;

  vda5050pp::misc::AnyPtr opaque_order_subscriber_;
  vda5050pp::misc::AnyPtr opaque_message_subscriber_;
  vda5050pp::misc::AnyPtr opaque_navigation_status_subscriber_;
  std::shared_ptr<vda5050pp::events::ScopedOrderEventSubscriber> order_event_subscriber_;

public:
  ///
  ///\brief Construct a new Order Observer object (requires a running instance)
  ///
  OrderObserver();

  ///
  ///\brief Get the last observed node id and sequence id
  ///
  ///\return std::pair<std::string, decltype(vda5050::Node::sequenceId)> the last observed node
  /// id and seq id
  ///
  std::optional<std::pair<std::string, decltype(vda5050::Node::sequenceId)>> getLastNode() const;

  ///
  ///\brief Return the last action status observed for the given action id
  ///
  ///\param action_id the action id
  ///\return const vda5050::ActionStatus& the last observed status for action_id
  ///
  std::optional<vda5050::ActionStatus> getActionStatus(std::string_view action_id) const;

  ///
  ///\brief Get the last observed OrderStatus
  ///
  ///\return vda5050pp::misc::OrderStatus the last observed OrderStatus
  ///
  std::optional<vda5050pp::misc::OrderStatus> getOrderStatus() const;

  ///
  ///\brief Get the last observed OrderId
  ///
  ///\return std::optional<std::pair<std::string, uint32_t>> the last observed OrderId
  ///
  std::optional<std::pair<std::string, uint32_t>> getOrderId() const;

  ///
  ///\brief Add a callback to be called when the order status changes
  ///
  ///\param callback the callback to be called
  ///
  void onOrderStatusChanged(std::function<void(vda5050pp::misc::OrderStatus)> &&callback);

  ///
  ///\brief Add a callback to be called when the order id changes
  ///
  ///\param callback the callback to be called
  ///
  void onOrderIdChanged(std::function<void(std::pair<std::string, uint32_t>)> &&callback);

  ///
  ///\brief Add a callback to be called when the action status changes
  ///
  ///\param action_id the action id to observe
  ///\param callback the callback to be called
  ///
  void onActionStatusChanged(std::string_view action_id,
                             std::function<void(vda5050::ActionStatus)> &&callback);

  ///
  ///\brief Add a callback to be called when any action changes. Returns all current action
  /// statuses.
  ///
  ///\param callback the callback to be called
  ///
  void onActionStatusChanged(
      std::function<void(const std::vector<std::shared_ptr<const vda5050::ActionState>> &)>
          &&callback) const;

  ///
  ///\brief Add a callback to be called when the last node id changes
  ///
  /// The callback function gets two parameters: the node id and the sequence id.
  /// If the sequence id is unset, the node id change was not part of an order.
  ///
  ///\param callback the callback to be called
  ///
  void onLastNodeIdChanged(
      std::function<
          void(std::pair<std::string_view, std::optional<decltype(vda5050::Node::sequenceId)>>)>
          &&callback);

  ///
  ///\brief Add a callback to be called when the base changed. Note, that this should be not used
  /// for navigation.
  /// Refer to navigation specific events and the BaseNavigationHandler instead.
  ///
  void onBaseChanged(std::function<void(const std::vector<std::shared_ptr<const vda5050::Node>> &,
                                        const std::vector<std::shared_ptr<const vda5050::Edge>> &)>
                         &&callback) const;

  ///
  ///\brief Add a callback to be called when the horizon changes.
  ///
  void onHorizonChanged(
      std::function<void(const std::vector<std::shared_ptr<const vda5050::Node>> &,
                         const std::vector<std::shared_ptr<const vda5050::Edge>> &)> &&callback)
      const;

  ///
  ///\brief Add a callback to be called when the errors changed
  ///
  void onErrorsChanged(std::function<void(const std::vector<vda5050::Error> &)> &&callback) const;
};

}  // namespace vda5050pp::observer

#endif  // PUBLIC_VDA5050_2B_2B_OBSERVER_ORDER_OBSERVER_H_
