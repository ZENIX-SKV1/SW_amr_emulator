// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_EVENTS_EVENT_CONTROL_BLOCKS_H_
#define VDA5050_2B_2B_CORE_EVENTS_EVENT_CONTROL_BLOCKS_H_

#include <functional>
#include <list>
#include <memory>
#include <mutex>

#include "vda5050++/core/generic_event_manager.h"
#include "vda5050++/events/event_type.h"

namespace vda5050pp::core::events {

///
///\brief The base-class of all event control blocks.
///
/// Event control blocks are used to implement complex event control flows. Over time
/// without blocking a thread.
///
/// The original purpose was to handle control instant actions, such as pause/resume/cancel
/// For this the interpreter can construct an EventControlBlock, which
/// Waits for a certain event, and the executes arbitrary code and/or waits for new events.
///
class EventControlBlock {
private:
  std::function<void()> teardown_;

public:
  virtual ~EventControlBlock() = default;

  ///\brief set a teardown function, which can be called to clean up the block.
  template <typename Teardown> void setTeardown(Teardown &&teardown) {
    this->teardown_ = std::forward<Teardown>(teardown);
  }

  ///\brief setup the event control block, i.e. subscribe to certain events.
  virtual void enable() = 0;

  ///\brief call the teardown function of this block.
  virtual void teardown();
};

///
///\brief A chain of control blocks to be enabled in sequence.
/// It utilizes the teardown function of each block to enable the next block.
/// Once the last block is teared-down, the chain is teared-down.
///
class EventControlChain : public EventControlBlock {
private:
  std::list<std::shared_ptr<EventControlBlock>> blocks_;

protected:
  ///
  ///\brief Enable the next block in the list.
  /// If the list was emptied, the chain is teared down.
  ///
  ///\throws VDA5050PPInvalidState if the chain is empty.
  ///
  void next() noexcept(false);

public:
  ///
  ///\brief Enable the first block in the chain.
  ///
  ///\throws VDA5050PPInvalidState if the chain is empty.
  ///
  void enable() noexcept(false) override;

  ///
  ///\brief Add a block to the chain (teardown will be overridden)
  ///
  ///\param block The block to add.
  ///
  ///\brief throws VDA5050PPNullPointer if block is nullptr.
  ///
  void add(std::shared_ptr<EventControlBlock> block) noexcept(false);
};

///
///\brief An alternative of multiple control blocks.
///
/// Once one of the alternative blocks is finished, i.e. teared-down,
/// all blocks are removed and the Alternative is teared-down.

///
class EventControlAlternative : public EventControlBlock {
private:
  std::vector<std::shared_ptr<EventControlBlock>> blocks_;
  bool teardown_called_ = false;

public:
  ///
  ///\brief Enable all alternative blocks.
  ///
  ///\throws VDA5050PPInvalidState if the alternative is empty.
  ///
  void enable() noexcept(false) override;

  ///
  ///\brief Add a new block to the alternative.
  ///
  /// Teardown will be overridden.
  ///
  ///\param block The block to add as alternative.
  ///
  ///\throws VDA5050PPNullPointer if block is nullptr.
  ///
  void add(std::shared_ptr<EventControlBlock> block) noexcept(false);
};

///
///\brief A parallel control block, which enables all blocks in parallel.
/// All blocks must have been teared-down, before the parallel block is teared-down.
///
class EventControlParallel : public EventControlBlock {
private:
  std::mutex blocks_mutex_;
  std::vector<std::shared_ptr<EventControlBlock>> blocks_;

public:
  ///
  ///\brief Enable all parallel blocks.
  ///
  void enable() noexcept(false) override;

  ///
  ///\brief Add a new block to the parallel block.
  ///
  /// Teardown will be overridden.
  ///
  ///\param block The new block to add.
  ///
  void add(std::shared_ptr<EventControlBlock> block) noexcept(false);
};

///
///\brief A latch control block. It calls the done and tears-down, once an event satisfies a
/// predicate.
///
/// It is meant to be used as a base-class for a concrete latch with
/// an overridden predicate and done function.
///
///\tparam EventT the specialized type of events to subscribe to.
///
template <typename EventT> class EventLatch : public EventControlBlock {
public:
  using Subscriber =
      typename vda5050pp::core::GenericEventManager<typename EventT::EventBase>::ScopedSubscriber;

private:
  Subscriber sub_;

public:
  ///
  ///\brief The predicate function to be overridden by the concrete latch.
  ///
  ///\param evt the received event.
  ///\return true if the predicate is satisfied.
  ///
  virtual bool predicate(std::shared_ptr<EventT> evt) = 0;

  ///
  ///\brief The done function to be overridden by the concrete latch.
  /// It is called, when the predicate returned true.
  ///
  virtual void done() = 0;

  ///
  ///\brief Construct a new EventLatch using a subscriber to use when enabled.
  ///
  ///\param sub the subscriber to use when enabled.
  ///
  explicit EventLatch(Subscriber &&sub) : sub_(std::move(sub)) {}

  ///
  ///\brief Enable this latch and setup the subscriber.
  ///
  void enable() override {
    this->sub_.template subscribe<EventT>([this](auto evt) {
      if (this->predicate(evt)) {
        this->done();
        this->teardown();
      }
    });
  }
};

///
///\brief An event latch with function objects instead of overridden functions.
///
/// It calls the done and tears-down, once an event satisfies a predicate.
///
///\tparam EventT The specialized type of events to subscribe to.
///
template <typename EventT> class LambdaEventLatch : public EventLatch<EventT> {
private:
  ///\brief the stored predicate function.
  std::function<bool(std::shared_ptr<EventT>)> pred_;

  ///\brief the stored done function.
  std::function<void()> done_;

public:
  ///
  ///\brief Overridden predicate function, which forwards to the stored predicate function.
  ///
  ///\param evt the event to check
  ///\return true if the predicate is satisfied.
  ///
  bool predicate(std::shared_ptr<EventT> evt) override { return this->pred_(evt); }

  ///
  ///\brief Overridden done function, which forwards to the stored done function.
  ///
  void done() override { this->done_(); }

  ///
  ///\brief Construct a new Lambda Event Latch object
  ///
  ///\tparam PredFn the type of the predicate function
  ///\tparam DoneFn the type of the done function
  ///\param sub the subscriber to use when enabled.
  ///\param pred the predicate function
  ///\param done the done function
  ///
  template <typename PredFn, typename DoneFn>
  LambdaEventLatch(typename EventLatch<EventT>::Subscriber &&sub, PredFn &&pred, DoneFn &&done)
      : EventLatch<EventT>(std::move(sub)),
        pred_(std::forward<PredFn>(pred)),
        done_(std::forward<DoneFn>(done)) {}
};

///
///\brief A function block, which calls a function when enabled and then tears-down.
///
class FunctionBlock : public EventControlBlock {
private:
  std::function<void()> fn_;

public:
  ///
  ///\brief Set the Function to call on enable.
  ///
  ///\tparam Fn the type of the function
  ///\param fn the function to call
  ///
  template <typename Fn> void setFunction(Fn &&fn) { this->fn_ = std::forward<Fn>(fn); }

  ///
  ///\brief Call the function if set and teardown.
  ///
  void enable() override;
};

}  // namespace vda5050pp::core::events

#endif  // VDA5050_2B_2B_CORE_EVENTS_EVENT_CONTROL_BLOCKS_H_
