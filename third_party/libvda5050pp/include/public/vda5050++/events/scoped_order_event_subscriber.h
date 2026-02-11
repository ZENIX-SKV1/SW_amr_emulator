//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PUBLIC_VDA5050_2B_2B_EVENTS_SCOPED_ORDER_EVENT_SUBSCRIBER_H_
#define PUBLIC_VDA5050_2B_2B_EVENTS_SCOPED_ORDER_EVENT_SUBSCRIBER_H_

#include <functional>
#include <memory>

#include "vda5050++/events/order_event.h"

namespace vda5050pp::events {

///
///\brief The scoped OrderEvent subscriber. This class can only be instantiated by the library.
/// As a user you can get an instance via the EventHandle. The subscribe functions can be set to
/// subscribe to library events. As the name implies, the lifetime of the subscription is RAII
/// based.
///
class ScopedOrderEventSubscriber {
public:
  virtual ~ScopedOrderEventSubscriber() = default;

  ///
  ///\brief Subscribe a OrderEvent.
  ///
  ///\param callback the handler to set.
  ///
  virtual void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OrderBaseChanged>)>
                             &&callback) noexcept(true) = 0;

  ///
  ///\brief Subscribe a OrderEvent.
  ///
  ///\param callback the handler to set.
  ///
  virtual void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::OrderHorizonChanged>)>
          &&callback) noexcept(true) = 0;

  ///
  ///\brief Subscribe a OrderEvent.
  ///
  ///\param callback the handler to set.
  ///
  virtual void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::OrderActionStatesChanged>)>
          &&callback) noexcept(true) = 0;

  ///
  ///\brief Subscribe a OrderEvent.
  ///
  ///\param callback the handler to set.
  ///
  virtual void subscribe(std::function<void(std::shared_ptr<vda5050pp::events::OrderErrorsChanged>)>
                             &&callback) noexcept(true) = 0;

  ///
  ///\brief Subscribe to OrderLastNodeChanged events.
  ///
  ///\param callback the handler to set
  ///
  virtual void subscribe(
      std::function<void(std::shared_ptr<vda5050pp::events::OrderLastNodeChanged>)>
          &&callback) noexcept(true) = 0;
};

}  // namespace vda5050pp::events

#endif  // PUBLIC_VDA5050_2B_2B_EVENTS_SCOPED_ORDER_EVENT_SUBSCRIBER_H_
