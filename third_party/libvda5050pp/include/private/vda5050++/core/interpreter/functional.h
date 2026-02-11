//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_INTERPRETER_FUNCTIONAL_H_
#define VDA5050_2B_2B_CORE_INTERPRETER_FUNCTIONAL_H_

#include <vda5050/Order.h>

#include <queue>
#include <tuple>

#include "vda5050++/core/events/interpreter_event.h"
#include "vda5050++/core/state/graph.h"

namespace vda5050pp::core::interpreter {

using NodeIter = std::vector<vda5050::Node>::const_iterator;
using EdgeIter = std::vector<vda5050::Edge>::const_iterator;
using ActionIter = std::vector<vda5050::Action>::const_iterator;

///
///\brief The event iterator is used to iterate over an order and yield an event each step.
///
class EventIter {
public:
  ///\brief Represents the current state of the iterator
  enum class IterState {
    ///\brief the initial state of the iterator
    /// If the order is replacing, dispatch YieldClearActions event
    /// Step to handling_pre_node_action or handling_transition
    k_handling_initial,
    ///\brief The step before handling actions on a node
    /// Setups action iterators
    /// Steps to handling_node_action or handling_transition
    k_handling_pre_node_action,
    ///\brief Handling actions on a node
    /// Inspects the current action in the iterator and steps the iterator.
    /// Steps to handling_node_action_queue or handling_pre_node_action
    /// Populates the current action group and sets stop_at_goal optionally
    k_handling_node_action,
    ///\brief Inserts the action group into the queue to do after navigation to this node
    /// Steps to handling_node_action or handling_transition
    k_handling_node_action_queue,
    ///\brief Prepare handling of edge actions
    /// Prepares action iterators
    /// Steps to handling_edge_action or handling_navigation
    k_handling_pre_edge_action,
    ///\brief Handling edge actions
    /// Inspects the current action in the iterator and steps the iterator.
    /// Steps to handling_edge_action_queue or handling_pre_edge_action
    /// Populates the current action group and sets stop_at_goal optionally
    k_handling_edge_action,
    ///\brief Adds the current action group to the queue to do before navigating via this edge
    /// Steps to handling_edge_action or handling_navigation
    k_handling_edge_action_queue,
    ///\brief Yields the action queue as events
    /// Steps to handling_pre_node_action or handling_edge_action_queue
    k_handling_action_yield,
    ///\brief Handling navigation
    /// Dispatches YieldNavigationEvent:
    /// Resets current goal/edge/stop_at_goal
    /// Steps to handling_action_yield
    k_handling_navigation,
    ///\brief Handling transition
    /// Steps node/edge iterators
    /// Adds to current graph
    /// Steps to pre_done or handling_pre_edge_action
    k_handling_transition,
    ///\brief yield the remaining data
    /// Dispatches YieldNavigationEvent, YieldGraphReplacement, YieldGraphExtension
    /// Steps to done or pre_done
    k_pre_done,
    ///\brief yields nullptr
    k_done,
  };

protected:
  ///
  ///\brief rw access to the current iter state
  ///
  ///\return IterState& current iter state
  ///
  IterState &getIterState();

  ///
  ///\brief rw access to the node iterator
  ///
  ///\return NodeIter& current node iterator
  ///
  NodeIter &getNodeIter();

  ///
  ///\brief rw access to the node iterator end
  ///
  ///\return NodeIter& current node iterator end
  ///
  NodeIter &getNodeEnd();

  ///
  ///\brief rw access to the edge iterator
  ///
  ///\return EdgeIter& the current edge iterator
  ///
  EdgeIter &getEdgeIter();

  ///
  ///\brief rw access to the edge iterator end
  ///
  ///\return EdgeIter& the current edge iterator end
  ///
  EdgeIter &getEdgeEnd();

  ///
  ///\brief rw access to the action iterator
  ///
  ///\return ActionIter& the current action iterator
  ///
  ActionIter &getActionIter();

  ///
  ///\brief rw access to the action iterator end
  ///
  ///\return ActionIter& the current action iterator end
  ///
  ActionIter &getActionEnd();

  ///
  ///\brief rw access to the current order id
  ///
  ///\return std::string& the current order id
  ///
  std::string &getOrderId();

  ///
  ///\brief rw access to the current order update id
  ///
  ///\return uint32_t& the current order update id
  ///
  uint32_t &getOrderUpdateId();

  ///
  ///\brief rw access to the collected graph
  ///
  ///\return std::shared_ptr<vda5050pp::core::state::Graph>& the current collected graph
  ///
  std::shared_ptr<vda5050pp::core::state::Graph> &getCollectedGraph();

  ///
  ///\brief rw access to the current goal node
  ///
  ///\return std::shared_ptr<vda5050::Node>& the current goal node
  ///
  std::shared_ptr<vda5050::Node> &getCurrentGoalNode();

  ///
  ///\brief rw access to the current via edge
  ///
  ///\return std::shared_ptr<vda5050::Edge>& the current via edge
  ///
  std::shared_ptr<vda5050::Edge> &getCurrentViaEdge();

  ///
  ///\brief rw access to the current action group
  ///
  ///\return std::vector<std::shared_ptr<const vda5050::Action>>& the current action group
  ///
  std::vector<std::shared_ptr<const vda5050::Action>> &getCurrentActionGroup();

  ///
  ///\brief rw access to the stop at goal flag
  ///
  ///\return bool& the current stop at goal flag
  ///
  bool &getStopAtGoal();

  ///
  ///\brief rw access to the current highest action group blocking type
  ///
  ///\return vda5050::BlockingType&  the current highest action group blocking type
  ///
  vda5050::BlockingType &getCurrentActionGroupBlockingType();

  ///
  ///\brief rw access to the action events after navigation
  ///
  ///\return std::queue<std::shared_ptr<vda5050pp::core::events::InterpreterEvent>>&  the current
  /// action events after navigation
  ///
  std::queue<std::shared_ptr<vda5050pp::core::events::InterpreterEvent>>
      &getActionEventsAfterNavigation();

  ///
  ///\brief Update the current highest action group blocking type
  ///
  /// NONE < SOFT < HARD
  ///
  ///\param additional_blocking_type the new blocking type to consider
  ///
  void ceilCurrentActionGroupBlockingType(vda5050::BlockingType additional_blocking_type);

  EventIter() = default;

private:
  ///\brief the current node iterator
  NodeIter node_iter_;
  ///\brief the current edge iterator
  EdgeIter edge_iter_;
  ///\brief the current node iterator end
  NodeIter node_end_;
  ///\brief the current edge iterator end
  EdgeIter edge_end_;
  ///\brief the current action iterator
  ActionIter action_iter_;
  ///\brief the current action iterator end
  ActionIter action_end_;

  ///\brief the current order id
  std::string order_id_;
  ///\brief the current order update id
  uint32_t order_update_id_ = 0;
  ///\brief the current collected graph
  std::shared_ptr<vda5050pp::core::state::Graph> collected_graph_;
  ///\brief the current goal node
  std::shared_ptr<vda5050::Node> current_goal_node_;
  ///\brief the current via edge
  std::shared_ptr<vda5050::Edge> current_via_edge_;
  ///\brief a flag to indicate a stop at the current goal node
  bool stop_at_goal_ = false;

  ///\brief the current action group
  std::vector<std::shared_ptr<const vda5050::Action>> current_action_group_;

  ///\brief the current highest action group blocking type
  vda5050::BlockingType current_action_group_blocking_type_;

  ///\brief the action events after navigation
  std::queue<std::shared_ptr<vda5050pp::core::events::InterpreterEvent>>
      action_events_after_navigation_;

  ///\brief the current iter state
  IterState iter_state_ = IterState::k_handling_initial;

public:
  ///
  /// Create a new event iterator for the given order.
  ///
  ///\param order the order to interpret
  ///\return std::unique_ptr<EventIter> a pointer to the event iterator
  ///
  static std::unique_ptr<EventIter> fromOrder(const vda5050::Order &order);
};

///
///\brief Step this interpreter iterator once. This means yield one event.
///
/// The yield events are in the order they shall be scheduled.
/// Once the interpretation is done, nullptr is yielded.
///
///\param event_iter the event iterator to step
///\return std::tuple<std::shared_ptr<vda5050pp::core::events::InterpreterEvent>,
/// std::unique_ptr<EventIter>>  returns the next event and the next iterator state
///
std::tuple<std::shared_ptr<vda5050pp::core::events::InterpreterEvent>, std::unique_ptr<EventIter>>
nextEvent(std::unique_ptr<EventIter> &&event_iter) noexcept(false);

}  // namespace vda5050pp::core::interpreter

#endif  // VDA5050_2B_2B_CORE_INTERPRETER_FUNCTIONAL_H_
