// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// This file contains the FactsheetEventHandler.
//
//

#ifndef VDA5050_2B_2B_CORE_FACTSHEET_FACTSHEET_EVENT_HANDLER_H_
#define VDA5050_2B_2B_CORE_FACTSHEET_FACTSHEET_EVENT_HANDLER_H_

#include "vda5050++/core/events/factsheet_event.h"
#include "vda5050++/core/module.h"

namespace vda5050pp::core::factsheet {

///
///\brief The FactsheetEventHandler module handles FactsheetGatherEvents and
/// gathers the factsheet data.
///
class FactsheetEventHandler : public vda5050pp::core::Module {
private:
  std::optional<vda5050pp::core::GenericEventManager<
      vda5050pp::core::events::FactsheetEvent>::ScopedSubscriber>
      subscriber_;

protected:
  ///>
  ///\brief Handles the FactsheetGatherEvents. The factsheet data is the filled and set in the
  /// result of the event.
  ///
  ///\param evt the event data
  ///
  ///\throws VDA5050PPInvalidEventData if the event data is empty
  ///
  void handleFactsheetGatherEvent(
      std::shared_ptr<vda5050pp::core::events::FactsheetGatherEvent> evt) const noexcept(false);

public:
  ///
  ///\brief Setup the subscriber
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Unsubscribe from the events
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get a description of the module (possibly deprecated)
  ///
  ///\return std::string_view the module's description
  ///
  std::string_view describe() const override;
};

}  // namespace vda5050pp::core::factsheet

#endif  // VDA5050_2B_2B_CORE_FACTSHEET_FACTSHEET_EVENT_HANDLER_H_
