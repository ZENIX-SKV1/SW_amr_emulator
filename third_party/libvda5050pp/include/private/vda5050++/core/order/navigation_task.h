//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_ORDER_NAVIGATION_TASK_H_
#define VDA5050_2B_2B_CORE_ORDER_NAVIGATION_TASK_H_

#include <spdlog/fmt/fmt.h>
#include <vda5050/Edge.h>
#include <vda5050/Node.h>

#include <memory>
#include <string>

namespace vda5050pp::core::order {

///
///\brief The NavigationTransition class represents a transition in the state machine of a
/// NavigationTask.
///
/// Note: Can only be constructed via the static member functions
///
class NavigationTransition {
public:
  enum class Type {
    k_do_start,
    k_is_resumed,
    k_do_pause,
    k_is_paused,
    k_do_resume,
    k_to_seq_id,
    k_do_cancel,
    k_is_failed,
  };

private:
  ///
  ///\brief The current type of the transition.
  ///
  Type type_;
  ///\brief An associated sequence id for toSeqId()
  decltype(vda5050::Node::sequenceId) seq_id_;

  NavigationTransition(Type type, decltype(vda5050::Node::sequenceId) seq_id);

public:
  ///
  ///\brief Transition from the waiting state to the first in progress state.
  ///
  ///\return NavigationTransition the transition
  ///
  static NavigationTransition doStart();

  ///
  ///\brief Transition from the resuming state to the in progress state
  ///
  ///\return NavigationTransition the transition
  ///
  static NavigationTransition isResumed();

  ///
  ///\brief Transition from the in progress state to the pausing state
  ///
  ///\return NavigationTransition the transition
  ///
  static NavigationTransition doPause();

  ///
  ///\brief Transition from the pausing state to the paused state
  ///
  ///\return NavigationTransition the transition
  ///
  static NavigationTransition isPaused();

  ///
  ///\brief Transition from the paused state to the resuming state
  ///
  ///\return NavigationTransition the transition
  ///
  static NavigationTransition doResume();

  ///
  ///\brief Transition from the in progress state to the done state. (Must match the seq_id of the
  /// current goal)
  ///
  ///\param seq_id the sequence id of the node that was reached.
  ///\return NavigationTransition the Transition
  ///
  static NavigationTransition toSeqId(decltype(vda5050::Node::sequenceId) seq_id);

  ///
  ///\brief Transition from any non-terminal state to the canceling state
  ///
  ///\return NavigationTransition the transition
  ///
  static NavigationTransition doCancel();

  ///
  ///\brief Transition to the failed state
  ///
  ///\return NavigationTransition the transition
  ///
  static NavigationTransition isFailed();

  ///
  ///\brief Get the type of the transition
  ///
  ///\return Type the transition type
  ///
  Type type() const;

  ///
  ///\brief Get the sequence id associated with the transition (only for toSeqId)
  ///
  ///\return decltype(vda5050::Node::sequenceId) the sequence id
  ///
  decltype(vda5050::Node::sequenceId) seqId() const;

  ///
  ///\brief Get a string representation of the transition
  ///
  ///\return std::string the description
  ///
  std::string describe() const;
};

class NavigationTask;

///
///\brief The abstract NavigationState represents the current state of a running navigation task.
///
class NavigationState {
private:
  ///\brief the associated task (state-machine), which owns this state.
  NavigationTask &task_;

protected:
  ///\brief get a reference to the associated task
  NavigationTask &task();

public:
  ///
  ///\brief Create a new Navigation state associated to the given task
  ///
  ///\param task the associated task
  ///
  explicit NavigationState(NavigationTask &task);
  virtual ~NavigationState() = default;

  ///
  ///\brief Transfer to a new state based on the given transition.
  /// Upon transitioning, the this state can be discarded
  ///
  ///\param transition the transition
  ///\return std::unique_ptr<NavigationState> the new state.
  ///
  [[nodiscard]] virtual std::unique_ptr<NavigationState> transfer(
      NavigationTransition transition) = 0;

  ///
  ///\brief an effect to run, when entering this state
  ///
  virtual void effect() = 0;

  ///
  ///\brief Check if the state is terminal state (done or failed)
  ///
  ///\return true if terminal, false otherwise
  ///
  virtual bool isTerminal() = 0;

  ///
  ///\brief Check if the state is failed
  ///
  ///\return true if failed, false otherwise
  ///
  virtual bool isFailed() = 0;

  ///
  ///\brief Check if the state is paused (paused or resuming)
  ///
  ///\return true if paused, false otherwise
  ///
  virtual bool isPaused() = 0;

  ///
  ///\brief Get a string representation of the state
  ///
  ///\return std::string the description
  ///
  virtual std::string describe() = 0;
};

///
///\brief A partial implementation of the NavigationState, which overrides isTerminal, isFailed and
/// isPaused
///
///\tparam terminal the return value for isTerminal
///\tparam failed the return value for isFailed
///\tparam paused the return value for isPaused
///
template <bool terminal, bool failed, bool paused> class NavigationStateT : public NavigationState {
public:
  explicit NavigationStateT(NavigationTask &task) : NavigationState(task) {}
  ~NavigationStateT() override = default;
  bool isTerminal() override { return terminal; }
  bool isFailed() override { return failed; }
  bool isPaused() override { return paused; }
};

class NavigationWaiting : public NavigationStateT<false, false, false> {
public:
  explicit NavigationWaiting(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationFirstInProgress : public NavigationStateT<false, false, false> {
public:
  explicit NavigationFirstInProgress(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationInProgress : public NavigationStateT<false, false, false> {
public:
  explicit NavigationInProgress(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationPausing : public NavigationStateT<false, false, false> {
public:
  explicit NavigationPausing(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationPaused : public NavigationStateT<false, false, true> {
public:
  explicit NavigationPaused(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationResuming : public NavigationStateT<false, false, true> {
public:
  explicit NavigationResuming(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationCanceling : public NavigationStateT<false, false, false> {
public:
  explicit NavigationCanceling(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationFailed : public NavigationStateT<true, true, false> {
public:
  explicit NavigationFailed(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

class NavigationDone : public NavigationStateT<true, false, false> {
public:
  explicit NavigationDone(NavigationTask &task);
  [[nodiscard]] std::unique_ptr<NavigationState> transfer(NavigationTransition transition) override;
  void effect() override;
  std::string describe() override;
};

///
///\brief The NavigationTask represents the current state of the associated navigation goal.
///
/// It is implemented using the state-machine pattern. Each state will have an effect,
/// that is run on entering that state. These effects will dispatch events for the state module,
/// such that it can represent the state of the navigation in the vda5050::State.
///
class NavigationTask {
public:
  friend class NavigationWaiting;
  friend class NavigationFirstInProgress;
  friend class NavigationInProgress;
  friend class NavigationPausing;
  friend class NavigationPaused;
  friend class NavigationResuming;
  friend class NavigationCanceling;
  friend class NavigationFailed;
  friend class NavigationDone;

private:
  ///\brief the current state of the state-machine
  std::unique_ptr<NavigationState> state_;
  ///\brief the current segment of the navigation task
  std::optional<std::pair<decltype(vda5050::Node::sequenceId), decltype(vda5050::Node::sequenceId)>>
      segment_;
  ///\brief the navigation goal associated with the task
  std::shared_ptr<const vda5050::Node> goal_;
  ///\brief the via-edge associated with the task
  std::shared_ptr<const vda5050::Edge> via_edge_;

protected:
  ///
  ///\brief Get the goal node of the navigation task
  ///
  ///\return std::shared_ptr<const vda5050::Node> the goal node
  ///
  std::shared_ptr<const vda5050::Node> getGoal() const;

  ///
  ///\brief Get the via edge of the navigation task
  ///
  ///\return std::shared_ptr<const vda5050::Edge> the via edge
  ///
  std::shared_ptr<const vda5050::Edge> getViaEdge() const;

  ///
  ///\brief Get the segment associated with the navigation task (if any)
  ///
  ///\return std::optional<std::pair<decltype(vda5050::Node::sequenceId),
  /// decltype(vda5050::Node::sequenceId)>> the segment
  ///
  std::optional<std::pair<decltype(vda5050::Node::sequenceId), decltype(vda5050::Node::sequenceId)>>
  getSegment();

public:
  ///
  ///\brief Construct a new NavigationTask for a via-edge goal-node pair
  ///
  ///\param goal the goal node
  ///\param via_edge the via edge leading to the goal
  ///
  NavigationTask(std::shared_ptr<const vda5050::Node> goal,
                 std::shared_ptr<const vda5050::Edge> via_edge);

  ///
  ///\brief Set an optional segment, which begins with this navigation task
  ///
  ///\param segment the segment
  ///
  void setSegment(
      std::pair<decltype(vda5050::Node::sequenceId), decltype(vda5050::Node::sequenceId)> segment);

  ///
  ///\brief Transition the state-machine with a transition.
  ///
  ///\param transition the transition
  ///
  void transition(NavigationTransition transition);

  ///
  ///\brief Is the state-machine in a terminal state? (done or failed)
  ///
  ///\return true if terminal, false otherwise
  ///
  bool isTerminal() const;

  ///
  ///\brief Is the state-machine in a failed state? (failed)
  ///
  ///\return true if failed, false otherwise
  ///
  bool isFailed() const;

  ///
  ///\brief Is the state-machine in a paused state? (paused or resuming)
  ///
  ///\return true if paused, false otherwise
  ///
  bool isPaused() const;

  ///
  ///\brief Get a string representation of the current state
  ///
  ///\return std::string the description
  ///
  std::string describe() const;
};

}  // namespace vda5050pp::core::order

template <>
struct fmt::formatter<vda5050pp::core::order::NavigationTransition>
    : fmt::formatter<fmt::string_view> {
  template <typename FormatContext>
  auto format(vda5050pp::core::order::NavigationTransition t, FormatContext &ctx) const {
    return fmt::formatter<fmt::string_view>::format(t.describe(), ctx);
  }
};

#endif  // VDA5050_2B_2B_CORE_ORDER_NAVIGATION_TASK_H_
