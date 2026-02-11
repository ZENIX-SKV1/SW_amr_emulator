// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// This file contains the declaration of the VisualizationTimer module
//
//

#ifndef VDA5050_2B_2B_CORE_STATE_VISUALIZATION_TIMER_H_
#define VDA5050_2B_2B_CORE_STATE_VISUALIZATION_TIMER_H_

#include "vda5050++/core/common/interruptable_timer.h"
#include "vda5050++/core/common/scoped_thread.h"
#include "vda5050++/core/module.h"

namespace vda5050pp::core::state {

///
///\brief The VisualizationTimer module is responsible for sending visualization updates
/// periodically.
///
class VisualizationTimer : public vda5050pp::core::Module {
private:
  vda5050pp::core::common::InterruptableTimer timer_;
  vda5050pp::core::common::ScopedThread<void()> thread_;
  std::chrono::system_clock::duration update_period_;

protected:
  ///
  ///\brief Send the visualization update, i.e. dispatch an visualization message event
  ///
  void sendVisualization() const;

  ///
  ///\brief The timerRoutine periodically wakes up and calls sendVisualization().
  ///
  ///\param stop_token if the stop token is set, the routine will stop
  ///
  void timerRoutine(vda5050pp::core::common::StopToken stop_token) const;

public:
  VisualizationTimer();
  ///
  ///\brief Start the visualization timer and setup subscriptions
  ///
  ///\param instance the current library instance
  ///
  void initialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Stops the visualization timer and cleans up subscriptions
  ///
  ///\param instance the current library instance
  ///
  void deinitialize(vda5050pp::core::Instance &instance) override;

  ///
  ///\brief Get the description of the module (possibly deprecated)
  ///
  ///\return std::string_view the description
  ///
  std::string_view describe() const override;

  ///
  ///\brief Generate a new sub-config for the module.
  ///
  ///\return std::shared_ptr<vda5050pp::config::ModuleSubConfig> the sub-config.
  ///
  std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateSubConfig() const override;
};

}  // namespace vda5050pp::core::state

#endif  // VDA5050_2B_2B_CORE_STATE_VISUALIZATION_TIMER_H_
