//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#include "vda5050++/sinks/control_sink.h"

#include <future>
#include <random>

#include "vda5050++/core/instance.h"

using namespace vda5050pp::sinks;

static std::string randomId() {
  // Generate a random 128-bit string
  static std::random_device rd;
  static std::mt19937_64 gen(rd());

  uint64_t random_number_1 = gen();
  uint64_t random_number_2 = gen();

  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << random_number_1 << random_number_2;

  return oss.str();
}

static vda5050::Action mkAction(std::string_view type, std::optional<std::string_view> reason) {
  vda5050::Action action;
  action.actionId = fmt::format("internal_{}_{}", type, randomId());
  action.actionType = type;
  action.actionDescription = fmt::format("{} initiated by libVDA5050++ implementation: {}", type,
                                         reason.value_or("<No reason provided>"));
  return action;
}

template <typename Predicate, typename Action>
void waitForOrderStatusAfter(Predicate &&predicate, Action &&action) {
  static_assert(
      vda5050pp::core::common::is_signature_v<Predicate, bool(vda5050pp::misc::OrderStatus)>);
  static_assert(vda5050pp::core::common::is_signature_v<Action, void()>);

  std::promise<void> barrier_promise;
  auto barrier = barrier_promise.get_future();

  auto sub = vda5050pp::core::Instance::ref().getOrderEventManager().getScopedSubscriber();
  sub.subscribe<vda5050pp::core::events::OrderStatus>(
      [&barrier_promise, &predicate,
       done = false](std::shared_ptr<vda5050pp::core::events::OrderStatus> evt) mutable {
        if (!done && evt != nullptr && predicate(evt->status)) {
          done = true;  // Ensure to only set the promise once
          barrier_promise.set_value();
        }
      });

  action();

  barrier.wait();
}

void ControlSink::initiatePause(std::optional<std::string_view> reason, bool blocking) const
    noexcept(false) {
  auto ia = std::make_shared<vda5050pp::core::events::ValidInstantActionMessageEvent>();
  ia->valid_instant_actions = std::make_shared<vda5050::InstantActions>();
  ia->valid_instant_actions->actions.push_back(mkAction("startPause", reason));

  if (blocking) {
    waitForOrderStatusAfter(
        [](vda5050pp::misc::OrderStatus status) {
          using namespace vda5050pp::misc;
          return status == OrderStatus::k_order_paused ||
                 status == OrderStatus::k_order_idle_paused;
        },
        [&ia] { vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(ia); });
  } else {
    vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(ia);
  }
}

void ControlSink::initiateResume(std::optional<std::string_view> reason, bool blocking) const
    noexcept(false) {
  auto ia = std::make_shared<vda5050pp::core::events::ValidInstantActionMessageEvent>();
  ia->valid_instant_actions = std::make_shared<vda5050::InstantActions>();
  ia->valid_instant_actions->actions.push_back(mkAction("stopPause", reason));

  if (blocking) {
    waitForOrderStatusAfter(
        [](vda5050pp::misc::OrderStatus status) {
          using namespace vda5050pp::misc;
          return !(status == OrderStatus::k_order_idle ||
                   status == OrderStatus::k_order_idle_paused ||
                   status == OrderStatus::k_order_resuming);
        },
        [&ia] { vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(ia); });
  } else {
    vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(ia);
  }
}

void ControlSink::initiateCancel(std::optional<std::string_view> reason, bool blocking) const
    noexcept(false) {
  auto ia = std::make_shared<vda5050pp::core::events::ValidInstantActionMessageEvent>();
  ia->valid_instant_actions = std::make_shared<vda5050::InstantActions>();
  ia->valid_instant_actions->actions.push_back(mkAction("cancelOrder", reason));

  if (blocking) {
    waitForOrderStatusAfter(
        [](vda5050pp::misc::OrderStatus status) {
          using namespace vda5050pp::misc;
          return status == OrderStatus::k_order_idle || status == OrderStatus::k_order_idle_paused;
        },
        [&ia] { vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(ia); });
  } else {
    vda5050pp::core::Instance::ref().getMessageEventManager().dispatch(ia);
  }
}
