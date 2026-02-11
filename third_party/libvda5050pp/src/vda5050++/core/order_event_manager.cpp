//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//
#include "vda5050++/core/order_event_manager.h"

#include <eventpp/utilities/argumentadapter.h>

#include <chrono>
#include <functional>

#include "vda5050++/core/common/exception.h"
#include "vda5050++/core/common/formatters.h"
#include "vda5050++/core/logger.h"

using namespace vda5050pp::core;
using namespace std::chrono_literals;

ScopedOrderEventSubscriber::ScopedOrderEventSubscriber(OrderEventQueue &queue) : remover_(queue) {}

void ScopedOrderEventSubscriber::subscribe(
    std::function<void(std::shared_ptr<vda5050pp::events::OrderBaseChanged>)>
        &&callback) noexcept(true) {
  this->remover_.appendListener(
      vda5050pp::events::OrderBaseChanged::id(),
      eventpp::argumentAdapter<void(std::shared_ptr<vda5050pp::events::OrderBaseChanged>)>(
          std::move(callback)));
}

void ScopedOrderEventSubscriber::subscribe(
    std::function<void(std::shared_ptr<vda5050pp::events::OrderHorizonChanged>)>
        &&callback) noexcept(true) {
  this->remover_.appendListener(
      vda5050pp::events::OrderHorizonChanged::id(),
      eventpp::argumentAdapter<void(std::shared_ptr<vda5050pp::events::OrderHorizonChanged>)>(
          std::move(callback)));
}

void ScopedOrderEventSubscriber::subscribe(
    std::function<void(std::shared_ptr<vda5050pp::events::OrderActionStatesChanged>)>
        &&callback) noexcept(true) {
  this->remover_.appendListener(
      vda5050pp::events::OrderActionStatesChanged::id(),
      eventpp::argumentAdapter<void(std::shared_ptr<vda5050pp::events::OrderActionStatesChanged>)>(
          std::move(callback)));
}

void ScopedOrderEventSubscriber::subscribe(
    std::function<void(std::shared_ptr<vda5050pp::events::OrderErrorsChanged>)>
        &&callback) noexcept(true) {
  this->remover_.appendListener(
      vda5050pp::events::OrderErrorsChanged::id(),
      eventpp::argumentAdapter<void(std::shared_ptr<vda5050pp::events::OrderErrorsChanged>)>(
          std::move(callback)));
}

void ScopedOrderEventSubscriber::subscribe(
    std::function<void(std::shared_ptr<vda5050pp::events::OrderLastNodeChanged>)>
        &&callback) noexcept(true) {
  this->remover_.appendListener(
      vda5050pp::events::OrderLastNodeChanged::id(),
      eventpp::argumentAdapter<void(std::shared_ptr<vda5050pp::events::OrderLastNodeChanged>)>(
          std::move(callback)));
}

void OrderEventManager::threadTask(vda5050pp::core::common::StopToken tkn) noexcept(true) {
  if (this->opts_.synchronous_event_dispatch) {
    // This processing loop is not needed
    return;
  }

  try {
    while (!tkn.stopRequested()) {
      if (this->order_event_queue_.processUntil([&tkn] { return tkn.stopRequested(); })) {
        // no pause
      } else {
        std::this_thread::sleep_for(10ms);
      }
    }
  } catch (const vda5050pp::VDA5050PPError &err) {
    getEventsLogger()->error("OrderEventManager caught an exception, while processing events:\n {}",
                             err);
    // TODO: Handle critical exceptions
  }
}

OrderEventManager::OrderEventManager(const vda5050pp::core::events::EventManagerOptions &opts)
    : opts_(opts),
      thread_(std::bind(std::mem_fn(&OrderEventManager::threadTask), this, std::placeholders::_1)) {
}

void OrderEventManager::dispatch(std::shared_ptr<vda5050pp::events::OrderEvent> data,
                                 bool synchronous) noexcept(false) {
  if (data == nullptr) {
    throw vda5050pp::VDA5050PPInvalidEventData(MK_EX_CONTEXT("event is nullptr"));
  }

  if (this->opts_.synchronous_event_dispatch || synchronous) {
    this->order_event_queue_.dispatch(data->getId(), data);
  } else {
    this->order_event_queue_.enqueue(data->getId(), data);
  }
}

std::shared_ptr<ScopedOrderEventSubscriber>
OrderEventManager::getScopedOrderEventSubscriber() noexcept(true) {
  class Constructible : public ScopedOrderEventSubscriber {
  public:
    explicit Constructible(OrderEventQueue &queue) : ScopedOrderEventSubscriber(queue) {}
    ~Constructible() override = default;
  };
  auto ret = std::make_shared<Constructible>(this->order_event_queue_);
  return ret;
}