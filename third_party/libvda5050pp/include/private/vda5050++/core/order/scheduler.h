//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_ORDER_SCHEDULER_H_
#define VDA5050_2B_2B_CORE_ORDER_SCHEDULER_H_

#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>

#include "vda5050++/core/order/action_task.h"
#include "vda5050++/core/order/navigation_task.h"

namespace vda5050pp::core::order {

class Scheduler;

///
///\brief The SchedulerStateType enum represents the current state of the scheduler.
///
enum class SchedulerStateType {
  ///\brief the scheduler is idle
  k_idle,
  ///\brief the scheduler is idle and paused
  k_idle_paused,
  ///\brief the scheduler is active (processing tasks)
  k_active,
  ///\brief the scheduler is canceling
  k_canceling,
  ///\brief the scheduler is canceling during a pause
  k_canceling_paused,
  ///\brief the scheduler is resuming
  k_resuming,
  ///\brief the scheduler is pausing
  k_pausing,
  ///\brief the scheduler is paused (while having tasks)
  k_paused,
  ///\brief the scheduler is in a failed state
  k_failed,
  ///\brief the scheduler is interrupting the current tasks with instant action tasks
  k_interrupting,
};

///
///\brief The abstract SchedulerState class represents the state of the scheduler.
///
class SchedulerState {
private:
  ///\brief the associated scheduler (state-machine) owning this state
  Scheduler &scheduler_;

protected:
  ///
  ///\brief Get a rw reference to the scheduler
  ///
  ///\return Scheduler& the scheduler
  ///
  Scheduler &scheduler();

public:
  ///
  ///\brief Construct a new SchedulerState for a scheduler
  ///
  ///\param scheduler the scheduler
  ///
  explicit SchedulerState(Scheduler &scheduler);
  virtual ~SchedulerState() = default;

  ///
  ///\brief Try to cancel the tasks
  ///
  /// Returns the new state and a flag if an update is required. This state can be discarded.
  ///
  ///\return std::pair<std::unique_ptr<SchedulerState>, bool> the new state and a flag if an update
  /// is required
  ///
  [[nodiscard]] virtual std::pair<std::unique_ptr<SchedulerState>, bool> cancel() = 0;

  ///
  ///\brief Try to pause the tasks
  ///
  /// Returns the new state and a flag if an update is required. This state can be discarded.
  ///
  ///\return std::pair<std::unique_ptr<SchedulerState>, bool> the new state and a flag if an update
  /// is required
  ///
  [[nodiscard]] virtual std::pair<std::unique_ptr<SchedulerState>, bool> pause() = 0;

  ///
  ///\brief Try to resume the tasks
  ///
  /// Returns the new state and a flag if an update is required. This state can be discarded.
  ///
  ///\return std::pair<std::unique_ptr<SchedulerState>, bool> the new state and a flag if an update
  /// is required
  ///
  [[nodiscard]] virtual std::pair<std::unique_ptr<SchedulerState>, bool> resume() = 0;

  ///
  ///\brief Try to interrupt the tasks
  ///
  /// Returns the new state and a flag if an update is required. This state can be discarded.
  ///
  ///\return std::pair<std::unique_ptr<SchedulerState>, bool> the new state and a flag if an update
  /// is required
  ///
  [[nodiscard]] virtual std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() = 0;

  ///
  ///\brief Try to update the tasks
  ///
  /// Returns the new state and a flag if an update is required. This state can be discarded.
  ///
  ///\return std::pair<std::unique_ptr<SchedulerState>, bool> the new state and a flag if an update
  /// is required
  ///
  [[nodiscard]] virtual std::pair<std::unique_ptr<SchedulerState>, bool> update() = 0;

  ///
  ///\brief Try to reset the scheduler. After a reset, all queues must be empty and the state must
  /// be IDLE
  ///
  ///\return std::pair<std::unique_ptr<SchedulerState>, bool> the new state and a flag if an update
  /// is required
  ///
  [[nodiscard]] virtual std::pair<std::unique_ptr<SchedulerState>, bool> reset() = 0;

  ///
  ///\brief Get a description of the state
  ///
  ///\return std::string the description
  ///
  virtual std::string describe() = 0;

  ///
  ///\brief Get the type of the current state
  ///
  ///\return SchedulerStateType the state type
  ///
  virtual SchedulerStateType getState() = 0;
};

///
///\brief A partial implementation of the SchedulerState. It overrides the getState method.
///
///\tparam type the return value of getState
///
template <SchedulerStateType type> class SchedulerStateID : public SchedulerState {
public:
  explicit SchedulerStateID(Scheduler &scheduler) : SchedulerState(scheduler) {}
  SchedulerStateType getState() override { return type; }
};

class SchedulerIdle : public SchedulerStateID<SchedulerStateType::k_idle> {
public:
  SchedulerIdle(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerIdlePaused : public SchedulerStateID<SchedulerStateType::k_idle_paused> {
public:
  SchedulerIdlePaused(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerActive : public SchedulerStateID<SchedulerStateType::k_active> {
public:
  SchedulerActive(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerCanceling : public SchedulerStateID<SchedulerStateType::k_canceling> {
public:
  SchedulerCanceling(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerCancelingPaused : public SchedulerStateID<SchedulerStateType::k_canceling_paused> {
public:
  SchedulerCancelingPaused(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerResuming : public SchedulerStateID<SchedulerStateType::k_resuming> {
public:
  SchedulerResuming(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerPausing : public SchedulerStateID<SchedulerStateType::k_pausing> {
public:
  SchedulerPausing(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerPaused : public SchedulerStateID<SchedulerStateType::k_paused> {
public:
  SchedulerPaused(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerFailed : public SchedulerStateID<SchedulerStateType::k_failed> {
public:
  SchedulerFailed(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

class SchedulerInterrupting : public SchedulerStateID<SchedulerStateType::k_interrupting> {
public:
  SchedulerInterrupting(Scheduler &scheduler, bool notify = false);
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> cancel() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> pause() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> resume() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> interrupt() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> update() override;
  [[nodiscard]] std::pair<std::unique_ptr<SchedulerState>, bool> reset() override;
  std::string describe() override;
};

///
///\brief The Scheduler used for scheduling the events yield by the interpreter.
///
/// It is implemented using the state-machine pattern.
///
class Scheduler {
public:
  friend class SchedulerIdle;
  friend class SchedulerIdlePaused;
  friend class SchedulerActive;
  friend class SchedulerResuming;
  friend class SchedulerPausing;
  friend class SchedulerPaused;
  friend class SchedulerFailed;
  friend class SchedulerCanceling;
  friend class SchedulerCancelingPaused;
  friend class SchedulerInterrupting;
  friend class SchedulerInterrupted;

private:
  using Lock = std::unique_lock<std::mutex>;

  ///\brief The mutex protecting the members of the scheduler
  std::mutex access_mutex_;
  ///\brief The current scheduler state
  std::unique_ptr<SchedulerState> state_;
  ///\brief The queue holding the *commited* events of the scheduler. The events are processed in
  /// order.
  std::deque<std::shared_ptr<vda5050pp::core::events::InterpreterEvent>> rcv_evt_queue_;
  ///\brief The queue holding all *un-commited* events. They need to be commited to the
  /// rcv_evt_queue
  std::deque<std::shared_ptr<vda5050pp::core::events::InterpreterEvent>> rcv_evt_queue_staging_;
  ///\brief The queue holding instant actions, which interrupt the normal queue.
  std::deque<std::shared_ptr<vda5050pp::core::events::YieldInstantActionGroup>>
      rcv_interrupt_queue_;
  ///\brief The blocking type of the current action (if any)
  vda5050::BlockingType current_action_blocking_type_ = vda5050::BlockingType::NONE;
  ///\brief The current segment of the navigation task
  std::optional<std::pair<decltype(vda5050::Node::sequenceId), decltype(vda5050::Node::sequenceId)>>
      current_segment_;
  ///\brief Hold all active (running/paused) ActionTask state-machines
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>> active_action_tasks_by_id_;
  ///\brief Hold all running ActionTask state-machines (disjoint with paused)
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>> running_action_tasks_by_id_;
  ///\brief Hold all paused ActionTask state-machines (disjoint with running)
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>> paused_action_tasks_by_id_;
  ///\brief Hold all instant action tasks that interrupt the navigation, once empty the navigation
  /// can resume
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>>
      nav_interrupting_action_tasks_by_id_;
  ///\brief The current navigation task state-machine
  std::shared_ptr<NavigationTask> navigation_task_;

  ///
  ///\brief Ensure the lock is owned by the calling thread.
  ///
  /// If not currently owned, this blocks until the lock is acquired.
  ///
  ///\param lock the optional lock, if not provided or not owned a new lock is created
  ///\return Lock an acquired lock
  ///
  inline Lock ensureLock(std::optional<Lock> &&lock) {
    if (!lock.has_value() || !lock->owns_lock() || lock->mutex() != &this->access_mutex_) {
      return Lock(this->access_mutex_);
    }

    return std::move(lock.value());
  }

protected:
  ///
  ///\brief Get a rw reference the active (running/paused) action tasks mapping
  ///
  ///\return std::map<std::string, std::shared_ptr<ActionTask>, std::less<>>& the mapping
  ///
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>> &getActiveActionTasksById();

  ///
  ///\brief Get a rw reference the running action tasks mapping (disjoint with paused)
  ///
  ///\return std::map<std::string, std::shared_ptr<ActionTask>, std::less<>>& the mapping
  ///
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>> &getRunningActionTasksById();

  ///
  ///\brief Get a rw reference the paused action tasks mapping (disjoint with running)
  ///
  ///\return std::map<std::string, std::shared_ptr<ActionTask>, std::less<>>& the mapping
  ///
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>> &getPausedActionTasksById();

  ///
  ///\brief Get a rw reference the action tasks interrupting the navigation
  ///
  ///\return std::map<std::string, std::shared_ptr<ActionTask>, std::less<>>& the mapping
  ///
  std::map<std::string, std::shared_ptr<ActionTask>, std::less<>> &
  getNavInterruptingActionTasksById();

  ///
  ///\brief Get the queue of received instant actions, which are ready to be processed
  ///
  ///\return const std::deque<std::shared_ptr<vda5050pp::core::events::YieldInstantActionGroup>>&
  /// the queue
  ///
  const std::deque<std::shared_ptr<vda5050pp::core::events::YieldInstantActionGroup>> &
  getRcvInterruptQueue() const;

  ///
  ///\brief Get a rw reference to the current navigation task
  ///
  ///\return std::shared_ptr<NavigationTask>& the navigation task (may be nullptr)
  ///
  std::shared_ptr<NavigationTask> &getNavigationTask();

  ///
  ///\brief Clear all queues (optionally keep the navigation tasks in the queue)
  ///
  /// Keeping the navigation tasks allows them to be finished later.
  ///
  ///\param keep_nav keep all navigation tasks in the queue
  ///
  void clearQueues(bool keep_nav = false);

  ///
  ///\brief Perform a reset of the scheduler. Clear all queues, reset state and reset AGV
  ///
  void doReset();

  ///
  ///\brief Update task mappings, move form running to paused and vice versa, remove finished/failed
  ///
  void updateTasks();

  ///
  ///\brief Drop all finished/failed from interrupt mapping and optionally resume navigation
  ///
  void updateTasksInterruptMapping();

  ///
  ///\brief Fetch the next event from the rcv_evt_queue_ and process it with updateFetchNext(<type>)
  ///
  ///\param no_actions if the scheduler should stop processing when encountering actions.
  ///
  void updateFetchNext(bool no_actions = false);

  ///
  ///\brief Process a YieldActionGroupEvent and pop it from the queue
  ///
  ///\param evt the event on top of the queue
  ///
  void updateFetchNext(std::shared_ptr<vda5050pp::core::events::YieldActionGroupEvent> evt);

  ///
  ///\brief Process a YieldNavigationStepEvent and pop it from the queue
  ///
  ///\param evt the event on top of the queue
  ///
  void updateFetchNext(std::shared_ptr<vda5050pp::core::events::YieldNavigationStepEvent> evt);

  ///
  ///\brief Upon extending the order, update the current segment if applicable.
  /// This may yield another NavigationUpcomingSegmentEvent.
  ///
  void doPatchSegment();

  ///
  ///\brief Interrupt actions, which are non-compatible with the blocking type of the interrupting
  /// actions
  ///
  ///
  void doInterrupt();

  ///
  ///\brief During interrupt, update running actions and execute the interrupting actions, if
  /// possible.
  ///
  void updateFetchNextInterrupt();

public:
  ///
  ///\brief Construct a new Scheduler
  ///
  Scheduler();

  ///
  ///\brief Cancel the current tasks
  ///
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void cancel(std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Pause the current tasks
  ///
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void pause(std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Resume the current tasks
  ///
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void resume(std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Update the current tasks
  ///
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void update(std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Reset the scheduler (forceful cancel)
  ///
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void reset(std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Get the current state of the scheduler
  ///
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  ///\return the current state of the scheduler
  ///
  SchedulerStateType getState(std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Transition the associated state-machine of an action and step the scheduler accordingly.
  ///
  ///\param action_id The id of the action to step
  ///\param transition The transition to apply
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void actionTransition(std::string_view action_id, const ActionTransition &transition,
                        std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Transition the associated state-machine of the navigation task and step the scheduler
  ///
  ///\param transition the transition to apply
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void navigationTransition(NavigationTransition transition,
                            std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Enqueue interrupting actions into the scheduler.
  ///
  ///\param evt The event containing the interrupting actions
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void enqueueInterruptActions(
      std::shared_ptr<vda5050pp::core::events::YieldInstantActionGroup> evt,
      std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Enqueue non-interrupting events into the staging queue if the interpreter.
  ///
  /// The events must be commited before they are processed.
  ///
  ///\param evt The event to enqueue
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void enqueue(std::shared_ptr<vda5050pp::core::events::InterpreterEvent> evt,
               std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Commit the staging queue to the main queue and allow them to be processed.
  ///
  ///\param lock an optional lock for the scheduler. If not provided, a new lock is created.
  ///
  void commitQueue(std::optional<Lock> lock = std::nullopt);

  ///
  ///\brief Describe the current state of the scheduler.
  ///
  ///\return std::string the description
  ///
  std::string describe() const;
};

}  // namespace vda5050pp::core::order

#endif  // VDA5050_2B_2B_CORE_ORDER_SCHEDULER_H_
