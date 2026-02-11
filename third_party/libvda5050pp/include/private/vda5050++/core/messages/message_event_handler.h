//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_MESSAGES_MESSAGE_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_MESSAGES_MESSAGE_EVENT_HANDLER_H_

#include <optional>

#include "vda5050++/core/module.h"

namespace vda5050pp::core::messages {

///
///\brief The MessageEventHandler module is responsible for handling incoming messages
/// received by the MQTTModule.
///
class MessageEventHandler : public vda5050pp::core::Module {
private:
  std::optional<GenericEventManager<vda5050pp::core::events::MessageEvent>::ScopedSubscriber>
      subscriber_;

  ///
  ///\brief Handle an OrderMessageEvent. Validate the message and then send it to the interpreter.
  ///
  ///\param evt the received order event
  ///
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///\throws VDA5050PPSynchronizedEventTimedOut if the validation event times out
  ///
  void handleOrderMessage(
      std::shared_ptr<const vda5050pp::core::events::ReceiveOrderMessageEvent> evt) const;

  ///
  ///\brief Handle an InstantActionsMessageEvent. Validate the message and then send it to the
  /// interpreter.
  ///
  ///\param evt the received instant actions event
  ///
  ///\throws VDA5050PPInvalidEventData if the event is empty
  ///\throws VDA5050PPSynchronizedEventTimedOut if the validation event times out
  ///
  void handleInstantActionsMessage(
      std::shared_ptr<const vda5050pp::core::events::ReceiveInstantActionMessageEvent> evt) const;

public:
  ///
  ///\brief Setup all subscribers.
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribe all subscribers.
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a brief description of this module (possibly deprecated)
  ///
  ///\return std::string_view the description
  ///
  std::string_view describe() const override;
};

}  // namespace vda5050pp::core::messages

#endif  // VDA5050_2B_2B_CORE_MESSAGES_MESSAGE_EVENT_HANDLER_H_
