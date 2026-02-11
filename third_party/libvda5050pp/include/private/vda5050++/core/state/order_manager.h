//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_STATE_ORDER_MANAGER_H_
#define VDA5050_2B_2B_CORE_STATE_ORDER_MANAGER_H_

#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "vda5050++/core/common/math/linear_path_length_calculator.h"
#include "vda5050++/core/state/graph.h"
#include "vda5050++/misc/order_status.h"
#include "vda5050/ActionState.h"
#include "vda5050/Error.h"
#include "vda5050/Info.h"
#include "vda5050/State.h"

namespace vda5050pp::core::state {

///
///\brief The OrderManager holds the data about the order which are relevant for the vda5050::State
///
class OrderManager {
private:
  ///\brief the mutex protecting the data
  mutable std::mutex mutex_;
  ///\brief the current order's id
  std::string order_id_;
  ///\brief the current order's update id
  uint32_t order_update_id_ = 0;
  ///\brief the id of the last reached node
  std::string last_node_id_;
  ///\brief the seqId of the last reached node
  decltype(vda5050::Node::sequenceId) last_node_sequence_id_ = 0;
  ///\brief the graph the agv still has to traverse
  std::optional<Graph> graph_;
  ///\brief the currently known actions
  std::map<std::string, std::shared_ptr<const vda5050::Action>, std::less<>> action_by_id_;
  ///\brief the states of the currently known actions
  std::map<std::string, std::shared_ptr<vda5050::ActionState>, std::less<>> action_state_by_id_;
  ///\brief the current status of the order
  vda5050pp::misc::OrderStatus order_status_ = vda5050pp::misc::OrderStatus::k_order_idle;

  ///
  ///\brief Check if a lock is invalid
  ///
  ///\param lock the lock to check
  ///\return true if the lock locks this->mutex_ and is owned
  ///
  inline bool invalidLock(const std::unique_lock<std::mutex> &lock) const {
    return lock.mutex() != &this->mutex_ || !lock.owns_lock();
  }

protected:
  ///
  ///\brief Add a new Action
  ///
  ///\param action the action to add
  ///\param lock the current lock for the OrderManager
  ///
  ///\throws VDA5050PPInvalidArgument if the lock is invalid
  ///\throws VDA5050PPNullPointer if the action is a nullptr
  ///
  void addNewAction(std::shared_ptr<const vda5050::Action> action,
                    const std::unique_lock<std::mutex> &lock) noexcept(false);

  ///
  ///\brief Extend the current graph
  ///
  ///\param extension the graph to extend the current graph with
  ///\param lock the current lock for the OrderManager
  ///\return std::pair<GraphElement::SequenceId, GraphElement::SequenceId> delta sequence ids (all
  /// new elements)
  ///
  ///\throws VDA5050PPInvalidArgument if the lock is invalid
  ///\throws VDA5050PPInvalidArgument if there is no current graph
  ///
  std::pair<GraphElement::SequenceId, GraphElement::SequenceId> extendGraph(
      Graph &&extension, const std::unique_lock<std::mutex> &lock) noexcept(false);

  ///
  ///\brief Extend the current graph (setting the order_update_id)
  ///
  ///\param extension the graph to extend the current graph with
  ///\param order_update_id the order update id to set
  ///\param lock the current lock for the OrderManager
  ///\return std::pair<GraphElement::SequenceId, GraphElement::SequenceId> delta sequence ids (all
  /// new elements)
  ///
  ///\throws VDA5050PPInvalidArgument if the lock is invalid
  ///\throws VDA5050PPInvalidArgument if there is no current graph
  ///
  ///
  std::pair<GraphElement::SequenceId, GraphElement::SequenceId> extendGraph(
      Graph &&extension, uint32_t order_update_id,
      const std::unique_lock<std::mutex> &lock) noexcept(false);

  ///
  ///\brief Replace the current graph with a new graph (for new orders)
  ///
  ///\param new_graph the new graph
  ///\param order_id the order_id associated with that update
  ///\param lock the lock for the OrderManager
  ///
  ///\return bool if the last_node_id changed (due to temporary nodes)
  ///
  ///\throws VDA5050PPInvalidArgument if the lock is invalid
  ///
  bool replaceGraph(Graph &&new_graph, std::string_view order_id,
                    const std::unique_lock<std::mutex> &lock) noexcept(false);

  ///
  ///\brief Just set the last node id without involving the graph.
  ///
  ///\param last_node_id the last node id to set
  ///\param lock the lock for the OrderManager
  ///
  ///\return true if the id changed
  ///
  ///\throws VDA5050PPInvalidArgument if the lock is invalid
  ///
  bool setAGVLastNodeId(std::string_view last_node_id,
                        const std::unique_lock<std::mutex> &lock) noexcept(false);

public:
  ///
  ///\brief Get the Order id and update id of the current order
  ///
  ///\return std::pair<std::string, uint32_t> the order id and update id pair
  ///
  std::pair<std::string, uint32_t> getOrderId() const;

  ///
  ///\brief Get the current status of the order (scheduler state)
  ///
  ///\return vda5050pp::misc::OrderStatus the order status
  ///
  vda5050pp::misc::OrderStatus getOrderStatus() const;

  ///
  ///\brief Set the current status of the order (scheduler state)
  ///
  ///\param status the new status
  ///
  void setOrderStatus(vda5050pp::misc::OrderStatus status);

  ///
  ///\brief Add a new action to the state
  ///
  ///\param action the action to add
  ///
  ///\throws VDA5050PPNullPointer if the action is a nullptr
  ///\throws VDA5050PPInvalidArgument if the action is already known
  ///
  void addNewAction(std::shared_ptr<const vda5050::Action> action) noexcept(false);

  ///
  ///\brief Get an action by it's id
  ///
  ///\param action_id the id of the action
  ///\return std::shared_ptr<const vda5050::Action> the found action
  ///
  ///\throws VDA5050PPInvalidArgument if the action is not known
  ///
  std::shared_ptr<const vda5050::Action> getAction(std::string_view action_id) noexcept(false);

  ///
  ///\brief Try to get an action by it's id
  ///
  ///\param action_id the id of the action
  ///\return std::shared_ptr<const vda5050::Action> the found action or nullptr
  ///
  std::shared_ptr<const vda5050::Action> tryGetAction(std::string_view action_id);

  ///
  ///\brief Get the state of an action by it's id
  ///
  ///\param action_id the id of the action
  ///\return std::shared_ptr<vda5050::ActionState> the found action state
  ///
  ///\return VDA5050PPInvalidArgument if the action is not known
  ///
  std::shared_ptr<vda5050::ActionState> getActionState(std::string_view action_id) noexcept(false);

  ///
  ///\brief Try to get the state of an action by it's id
  ///
  ///\param action_id the id of the action
  ///\return std::shared_ptr<vda5050::ActionState> the found action state or nullptr
  ///
  std::shared_ptr<vda5050::ActionState> tryGetActionState(std::string_view action_id);

  ///
  ///\brief Get all action states
  ///
  ///\return std::vector<std::shared_ptr<const vda5050::ActionState>> all action states
  ///
  std::vector<std::shared_ptr<const vda5050::ActionState>> getActionStates() const;

  ///
  ///\brief Extend the current graph with a new graph. The graphs must stitch together, as in
  /// vda5050.
  ///
  ///\param extension the extending graph
  ///\return std::pair<GraphElement::SequenceId, GraphElement::SequenceId> the newly added range of
  /// sequence ids
  ///
  ///\throws VDA5050PPInvalidArgument if there is no current graph
  ///
  std::pair<GraphElement::SequenceId, GraphElement::SequenceId> extendGraph(
      Graph &&extension) noexcept(false);

  ///
  ///\brief Extend the current graph with a new graph. The graphs must stitch together, as in
  /// vda5050.
  /// Also sets the update-id associated with the extension
  ///
  ///\param extension the extending graph
  ///\param order_update_id the order update id to set for the extension
  ///\return std::pair<GraphElement::SequenceId, GraphElement::SequenceId> the newly added range of
  /// sequence ids
  ///
  ///\throws VDA5050PPInvalidArgument if there is no current graph
  ///
  std::pair<GraphElement::SequenceId, GraphElement::SequenceId> extendGraph(
      Graph &&extension, uint32_t order_update_id) noexcept(false);

  ///
  ///\brief Replace the current graph with a new one
  ///
  ///\param new_graph the new graph
  ///\param order_id the order_id associated with that graph
  ///
  ///\return bool if the last_node_id changed (due to temporary nodes)
  ///
  bool replaceGraph(Graph &&new_graph, std::string_view order_id) noexcept(false);

  ///
  ///\brief Get the current graph
  ///
  ///\return const Graph& the graph
  ///
  ///\throws VDA5050PPInvalidArgument if there is no current graph
  ///
  const Graph &getCurrentGraph() const noexcept(false);

  ///
  ///\brief Check if there is a graph in the state
  ///
  ///\return true if there is a graph
  ///
  bool hasGraph() const;

  ///
  ///\brief Set the last reached node of the AGV (based on the graph)
  ///
  ///\param seq_id the sequence id of the node to be found in the graph
  ///
  ///\throws VDA5050PPInvalidArgument if there is no current graph
  ///\throws VDA5050PPInvalidArgument if the sequence id does not belong to the graph
  ///\throws VDA5050PPInvalidArgument if the sequence id does not belong to a node
  ///
  void setAGVLastNode(uint32_t seq_id) noexcept(false);

  ///
  ///\brief Just set the last node id without involving the graph.
  ///
  ///\param last_node_id the last node id to set
  ///\return true if the id changed
  ///
  bool setAGVLastNodeId(std::string_view last_node_id) noexcept(false);

  ///
  ///\brief Get the last node id of the AGV without involving the graph
  ///
  ///\return std::string the last node id
  ///
  std::string getAGVLastNodeId() const noexcept(false);

  ///
  ///\brief Get the sequenceId of the last reached node
  ///
  ///\return decltype(vda5050::Node::sequenceId) the sequence id
  ///
  decltype(vda5050::Node::sequenceId) getAGVLastNodeSequenceId() const noexcept(true);

  ///
  ///\brief Dump the current order related data to a vda5050::State
  ///
  ///\param state the state to write to
  ///
  void dumpTo(vda5050::State &state) const;

  ///
  ///\brief Clear the current graph
  ///
  void clearGraph();

  ///
  ///\brief Set all actions, that are in the WAITING state to FAILED
  ///
  void cancelWaitingActions();

  ///
  ///\brief Clear all known actions (and states)
  ///
  void clearActions();
};

}  // namespace vda5050pp::core::state

#endif  // VDA5050_2B_2B_CORE_STATE_ORDER_MANAGER_H_
