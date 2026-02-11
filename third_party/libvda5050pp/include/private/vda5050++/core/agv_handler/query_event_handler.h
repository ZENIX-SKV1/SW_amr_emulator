// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// This file contains the QueryEventHandler.
//
//

#ifndef VDA5050_2B_2B_CORE_AGV_HANDLER_QUERY_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_AGV_HANDLER_QUERY_EVENT_HANDLER_H_

#include <optional>

#include "vda5050++/core/instance.h"
#include "vda5050++/core/module.h"

namespace vda5050pp::core::agv_handler {

///
///\brief The QueryEventHandler module is responsible for handling query events and
/// forwarding them to the registered user QueryHandler.
///
/// Default answers for the queries can be defined in the module's sub config.
///
class QueryEventHandler : public vda5050pp::core::Module {
private:
  std::optional<vda5050pp::core::ScopedQueryEventSubscriber> subscriber_;

  ///
  ///\brief Handle a QueryPauseable event.
  ///
  /// If a QueryEventHandler with overridden queryPauseable is set, the event is forwarded to the
  /// handler. Otherwise the default answer in the config is used.
  ///
  ///\param data the event data
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///
  void handleQueryPauseableEvent(std::shared_ptr<vda5050pp::events::QueryPauseable> data) const
      noexcept(false);

  ///
  ///\brief Handle a QueryResumable event.
  ///
  /// If a QueryEventHandler with overridden queryResumable is set, the event is forwarded to the
  /// handler. Otherwise the default answer in the config is used.
  ///
  ///\param data the event data
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///
  void handleQueryResumableEvent(std::shared_ptr<vda5050pp::events::QueryResumable> data) const
      noexcept(false);

  ///
  ///\brief Handle a QueryAcceptZoneSet event.
  ///
  /// If a QueryEventHandler with overridden queryAcceptZoneSet is set, the event is forwarded to
  /// the handler. Otherwise the default answer in the config is used.
  ///
  ///\param data the event data
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///
  void handleQueryAcceptZoneSet(std::shared_ptr<vda5050pp::events::QueryAcceptZoneSet> data) const
      noexcept(false);

  ///
  ///\brief Handle a QueryNodeTriviallyReachable event.
  ///
  /// If a QueryEventHandler with overridden queryNodeTriviallyReachable is set, the event is
  /// forwarded to the handler. Otherwise the default answer mechanism in the config is used.
  ///
  ///\param data the event data
  ///\throws VDA5050PPInvalidEventData when data is empty
  ///
  void handleQueryNodeTriviallyReachable(
      std::shared_ptr<vda5050pp::events::QueryNodeTriviallyReachable> data) const noexcept(false);

public:
  ///
  ///\brief Setup all subscribers
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribe all subscribers
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a brief module description (possibly deprecated)
  ///
  ///\return std::string_view a description
  ///
  std::string_view describe() const override;

  ///
  ///\brief Create a new QueryEventHandlerSubConfig instance
  ///
  ///\return std::shared_ptr<vda5050pp::config::ModuleSubConfig> the created subconfig
  ///
  std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateSubConfig() const override;
};

}  // namespace vda5050pp::core::agv_handler

#endif  // VDA5050_2B_2B_CORE_AGV_HANDLER_QUERY_EVENT_HANDLER_H_
