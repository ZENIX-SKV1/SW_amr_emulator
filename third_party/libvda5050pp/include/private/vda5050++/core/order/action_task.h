//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_ORDER_ACTION_TASK_H_
#define VDA5050_2B_2B_CORE_ORDER_ACTION_TASK_H_

#include <spdlog/fmt/fmt.h>
#include <vda5050/Action.h>

#include <memory>
#include <string>

#include "vda5050++/core/instance.h"

namespace vda5050pp::core::order {

///
///\brief The ActionTransition class represents a transition in the state machine of an action.
///
/// Note: Can only be constructed via the static member functions
///
class ActionTransition {
public:
  ///
  ///\brief The Transition Type
  ///
  enum class Type {
    k_do_start,
    k_is_initializing,
    k_is_running,
    k_do_pause,
    k_is_paused,
    k_do_resume,
    k_is_failed,
    k_is_finished,
    k_do_cancel,
  };

private:
  Type type_;
  std::optional<std::string> result_;

protected:
  ActionTransition(Type t, std::optional<std::string_view> result) : type_(t), result_(result) {}

public:
  ///
  ///\brief Create a transition from WAITING to INITIALIZING
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition doStart();

  ///
  ///\brief Transition from INITIALIZING to INITIALIZING (without effect)
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition isInitializing();

  ///
  ///\brief Transition from INITIALIZING to RUNNING
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition isRunning();

  ///
  ///\brief Transition from INITIALIZING or RUNNING to PAUSING
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition doPause();

  ///
  ///\brief Transition from PAUSING to PAUSED
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition isPaused();

  ///
  ///\brief Transition from PAUSED to RESUMING
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition doResume();

  ///
  ///\brief Transition from anywhere to FAILED
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition isFailed();

  ///
  ///\brief Transition from anywhere to FINISHED (with an optional result string)
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition isFinished(std::optional<std::string_view> result = std::nullopt);

  ///
  ///\brief Transition to CANCELING
  ///
  ///\return ActionTransition the transition
  ///
  static ActionTransition doCancel();

  ///
  ///\brief Get the Type of this transition
  ///
  ///\return Type the transition type
  ///
  Type getType() const;

  ///
  ///\brief Get the Result
  ///
  ///\return std::optional<std::string_view> the result
  ///
  std::optional<std::string_view> getResult() const;
};

class ActionTask;

///
///\brief The abstract ActionState represents the current state of a running action task.
///
class ActionState {
private:
  ///\brief The associated task, i.e. the State-Machine, which is the owner of this state.
  ActionTask &task_;

protected:
  ///
  ///\brief Get a reference to the associated ActionTask
  ///
  ///\return ActionTask& the associated ActionTask
  ///
  ActionTask &task();

public:
  ///
  ///\brief Construct a new ActionState object for a task
  ///
  ///\param task the ActionTask
  ///
  explicit ActionState(ActionTask &task);
  virtual ~ActionState() = default;

  ///
  ///\brief Return the new state after doing a transition from this state.
  ///
  /// After transitioning, this state object can be discarded.
  ///
  ///\param transition the transition to apply
  ///\return std::unique_ptr<ActionState> the new state
  ///
  [[nodiscard]] virtual std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) = 0;

  ///
  ///\brief Check if this state is a terminal state of the state-machine (FINISHED/FAILED)
  ///
  ///\return true if terminal, false otherwise
  ///
  virtual bool isTerminal() = 0;

  ///
  ///\brief Check if this state is a paused state of the state-machine (PAUSED/RESUMING)
  ///
  ///\return true if paused, false otherwise
  ///
  virtual bool isPaused() = 0;

  ///
  ///\brief The effect of this state. It will be executed on entering the state.
  ///
  virtual void effect() = 0;

  ///
  ///\brief Describe this state, i.e. return a human-readable string representation.
  ///
  ///\return std::string the state's description
  ///
  virtual std::string describe() = 0;
};

///
///\brief A Partial implementation of the ActionState, which overrides the isTerminal and isPaused
/// methods.
///
///\tparam terminal the return value of isTerminal
///\tparam paused  the return value of isPaused
///
template <bool terminal, bool paused> class ActionStateT : public ActionState {
public:
  explicit ActionStateT(ActionTask &task) : ActionState(task) {}
  ~ActionStateT() override = default;
  bool isTerminal() override { return terminal; }
  bool isPaused() override { return paused; }
};

class ActionWaiting : public ActionStateT<false, false> {
public:
  explicit ActionWaiting(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionInitializing : public ActionStateT<false, false> {
public:
  explicit ActionInitializing(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionInitializingNoEffect : public ActionStateT<false, false> {
public:
  explicit ActionInitializingNoEffect(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionRunning : public ActionStateT<false, false> {
public:
  explicit ActionRunning(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionPausing : public ActionStateT<false, false> {
public:
  explicit ActionPausing(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionResuming : public ActionStateT<false, true> {
public:
  explicit ActionResuming(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionPaused : public ActionStateT<false, true> {
public:
  explicit ActionPaused(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionCanceling : public ActionStateT<false, false> {
public:
  explicit ActionCanceling(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionFailed : public ActionStateT<true, false> {
public:
  explicit ActionFailed(ActionTask &task);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

class ActionFinished : public ActionStateT<true, false> {
private:
  std::optional<std::string> result_;

public:
  explicit ActionFinished(ActionTask &task, std::optional<std::string_view> result);
  [[nodiscard]] std::unique_ptr<ActionState> transition(
      const ActionTransition &transition) override;
  void effect() override;
  std::string describe() override;
};

///
///\brief The ActionTask represents the current state of the associated action.
///
/// It is implemented using the state-machine pattern. Each state will have an effect,
/// that is run on entering that state. These effects will dispatch events for the state module,
/// such that it can represent the state of the action in the vda5050::State.
///
class ActionTask {
public:
  friend class ActionWaiting;
  friend class ActionInitializing;
  friend class ActionRunning;
  friend class ActionPausing;
  friend class ActionResuming;
  friend class ActionPaused;
  friend class ActionCanceling;
  friend class ActionFailed;
  friend class ActionFinished;

private:
  ///\brief the current state of the state-machine
  std::unique_ptr<ActionState> state_;
  ///\brief the associated action
  std::shared_ptr<const vda5050::Action> action_;

public:
  ///
  ///\brief Create a new ActionTask for an action. Stats in the WAITING state.
  ///
  ///\param action the associated action
  ///
  explicit ActionTask(std::shared_ptr<const vda5050::Action> action);

  ///
  ///\brief Transition the state-machine. Each transition may yield some events
  ///
  ///\param transition the transition
  ///
  void transition(const ActionTransition &transition);

  ///
  ///\brief Check if the state-machine is in a terminal state (FINISHED/FAILED)
  ///
  ///\return true if terminal, false otherwise
  ///
  bool isTerminal() const;

  ///
  ///\brief Check if the state-machine is in a paused state (PAUSED/RESUMING)
  ///
  ///\return true if paused, false otherwise
  ///
  bool isPaused() const;

  ///
  ///\brief Get a reference to the associated action
  ///
  ///\return const vda5050::Action& the associated action
  ///
  const vda5050::Action &getAction() const;

  ///
  ///\brief Get a description of the current state
  ///
  ///\return std::string the current state's description
  ///
  std::string describe() const;
};

}  // namespace vda5050pp::core::order

template <>
struct fmt::formatter<vda5050pp::core::order::ActionTransition> : fmt::formatter<fmt::string_view> {
  template <typename FormatContext>
  auto format(const vda5050pp::core::order::ActionTransition &t, FormatContext &ctx) const {
    fmt::string_view name;
    switch (t.getType()) {
      case vda5050pp::core::order::ActionTransition::Type::k_do_start:
        name = "k_do_start";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_is_initializing:
        name = "k_is_initializing";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_is_running:
        name = "k_is_running";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_do_pause:
        name = "k_do_pause";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_is_paused:
        name = "k_is_paused";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_do_resume:
        name = "k_do_resume";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_is_failed:
        name = "k_is_failed";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_is_finished:
        name = "k_is_finished";
        break;
      case vda5050pp::core::order::ActionTransition::Type::k_do_cancel:
        name = "k_do_cancel";
        break;
      default:
        name = "unknown";
        break;
    }
    return fmt::formatter<fmt::string_view>::format(name, ctx);
  }
};

#endif  // VDA5050_2B_2B_CORE_ORDER_ACTION_TASK_H_
