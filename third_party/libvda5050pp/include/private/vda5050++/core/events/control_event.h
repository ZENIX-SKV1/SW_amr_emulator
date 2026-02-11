//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_EVENTS_CONTROL_EVENT_H_
#define VDA5050_2B_2B_CORE_EVENTS_CONTROL_EVENT_H_

#include "vda5050++/events/event_type.h"
#include "vda5050++/events/synchronized_event.h"

namespace vda5050pp::core::events {

///
///\brief The ControlEvent identifier type
///
enum class ControlEventType {
  k_control_messages,
};

///
///\brief The ControlEvent type is for events, that control the library.
/// Currently it only covers the connection of the Message Module.
///
struct ControlEvent : public vda5050pp::events::Event<ControlEventType> {};

///
///\brief Control the connection of the Message Module.
///
struct ControlMessagesEvent
    : public vda5050pp::events::EventId<ControlEvent, ControlEventType::k_control_messages>,
      vda5050pp::events::SynchronizedEvent<void> {
  ///\brief The type of the event.
  enum class Type {
    ///\brief connect the Message Module
    k_connect,
    ///\brief disconnect the Message Module
    k_disconnect,
  };
  ///\brief The type of the event.
  Type type;
};

}  // namespace vda5050pp::core::events

#endif  // VDA5050_2B_2B_CORE_EVENTS_CONTROL_EVENT_H_
