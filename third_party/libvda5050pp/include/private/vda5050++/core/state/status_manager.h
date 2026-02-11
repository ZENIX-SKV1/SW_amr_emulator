//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_STATE_STATUS_MANAGER_H_
#define VDA5050_2B_2B_CORE_STATE_STATUS_MANAGER_H_

#include <vda5050/BatteryState.h>
#include <vda5050/Error.h>
#include <vda5050/Info.h>
#include <vda5050/Load.h>
#include <vda5050/OperatingMode.h>
#include <vda5050/SafetyState.h>
#include <vda5050/State.h>

#include <cstdint>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>

#include "vda5050++/core/common/type_traits.h"

namespace vda5050pp::core::state {

///
///\brief The StatusManager holds the data about the the general status of the AGV
/// which are relevant for the vda5050::State
///
class StatusManager {
private:
  ///\brief the mutex protecting the data
  mutable std::shared_mutex mutex_;

  ///\brief all loads of the AGV
  std::optional<std::vector<vda5050::Load>> loads_;
  ///\brief the flag if a new base request is active
  std::optional<bool> new_base_request_;

  ///\brief The current battery state of the AGV
  vda5050::BatteryState battery_state_;
  ///\brief The current operating mode of the AGV
  vda5050::OperatingMode operating_mode_ = vda5050::OperatingMode::SERVICE;

  ///\brief The current errors of the AGV
  std::vector<vda5050::Error> errors_;

  ///\brief The current informations of the AGV
  std::vector<vda5050::Info> information_;

  ///\brief the current safety state of the AGV
  vda5050::SafetyState safety_state_;

  ///\brief the current position of the AGV
  vda5050::AGVPosition agv_position_;

  ///\brief the current velocity of the AGV
  std::optional<vda5050::Velocity> velocity_;

  ///\brief the current driving state of the AGV
  bool driving_ = false;

  ///\brief the distance since the last node of the AGV
  std::optional<double> distance_since_last_node_;

public:
  ///
  ///\brief Set the current AGV position
  ///
  ///\param agv_position the agv position
  ///
  void setAGVPosition(const vda5050::AGVPosition &agv_position);

  ///
  ///\brief Get the current AGV position
  ///
  ///\return vda5050::AGVPosition the current AGV position
  ///
  vda5050::AGVPosition getAGVPosition();

  ///
  ///\brief Set the current velocity
  ///
  ///\param velocity the velocity
  ///
  void setVelocity(const vda5050::Velocity &velocity);

  ///
  ///\brief Reset the currently store velocity
  ///
  void resetVelocity();

  ///
  ///\brief Get the Velocity (if set)
  ///
  ///\return std::optional<vda5050::Velocity> the velocity of std::nullopt
  ///
  std::optional<vda5050::Velocity> getVelocity() const;

  ///
  ///\brief Set the driving flag of the AGV
  ///
  ///\param driving is the agv driving?
  ///\return true, if the driving flag changed
  ///
  bool setDriving(bool driving);

  ///
  ///\brief Set the distance since the last node as in vda5050
  ///
  ///\param distance_since_last_node the new distance since the last node
  ///
  void setDistanceSinceLastNode(double distance_since_last_node);

  ///
  ///\brief Reset the distance since the last node
  ///
  void resetDistanceSinceLastNode();

  ///
  ///\brief Add a new load to the state
  ///
  ///\param load  the load to add
  ///\return true if the loads changed (in this case always)
  ///
  bool addLoad(const vda5050::Load &load);

  ///
  ///\brief Remove a load by it's id from the loads array
  ///
  ///\param load_id the id of the load to remove
  ///\return true if at least one load was removed
  ///
  bool removeLoad(std::string_view load_id);

  ///
  ///\brief Get the current loads
  ///
  ///\return const std::vector<vda5050::Load>& the current loads
  ///
  const std::vector<vda5050::Load> &getLoads();

  ///
  ///\brief Set the current operating mode of the AGV
  ///
  ///\param operating_mode the new operating mode
  ///\return true if the operating mode changed
  ///
  bool setOperatingMode(vda5050::OperatingMode operating_mode);

  ///
  ///\brief Get the current operating mode from the state
  ///
  ///\return vda5050::OperatingMode the current operating mode
  ///
  vda5050::OperatingMode getOperatingMode();

  ///
  ///\brief Set the current battery state of the AGV
  ///
  ///\param battery_state the battery state
  ///
  void setBatteryState(const vda5050::BatteryState &battery_state);

  ///
  ///\brief Get the current battery state from the state
  ///
  ///\return const vda5050::BatteryState& the current battery state
  ///
  const vda5050::BatteryState &getBatteryState();

  ///
  ///\brief Set the current safety state of the AGV
  ///
  ///\param safety_state the safety state
  ///
  ///\return true if the state changed
  ///
  bool setSafetyState(const vda5050::SafetyState &safety_state);

  ///
  ///\brief Get the current safety state from the state
  ///
  ///\return const vda5050::SafetyState& the current safety state
  ///
  const vda5050::SafetyState &getSafetyState();

  ///
  ///\brief Set the request new base flag to true
  ///
  void requestNewBase();

  ///
  ///\brief Add an error to the state
  ///
  ///\param error the error to add
  ///\return true if the errors array changed (in this case always)
  ///
  bool addError(const vda5050::Error &error);

  ///
  ///\brief Get a copy of the current errors
  ///
  ///\return std::vector<vda5050::Error>
  ///
  std::vector<vda5050::Error> getErrors() const;

  ///
  ///\brief Add a new information to the state
  ///
  ///\param info the information to add
  ///
  void addInfo(const vda5050::Info &info);

  /// \brief Alter the loads vector
  /// \tparam FunctionT callable void(std::vector<vda5050::Load> &)
  /// \param alter_function the alter_function
  template <typename FunctionT> bool loadsAlter(FunctionT alter_function) {
    static_assert(
        vda5050pp::core::common::is_signature<FunctionT, void(std::vector<vda5050::Load> &)>::value,
        "Expected type void(std::vector<vda5050::Load> &)");

    std::unique_lock lock(this->mutex_);

    if (!this->loads_.has_value()) {
      return false;
    }

    alter_function(*this->loads_);
    return true;
  }

  /// \brief Alter the operating mode
  /// \tparam FunctionT callable vda5050::OperatingMode(vda5050::OperatingMode)
  /// \param alter_function
  template <typename FunctionT> bool operatingModeAlter(FunctionT alter_function) {
    static_assert(
        vda5050pp::core::common::is_signature<FunctionT, vda5050::OperatingMode(
                                                             vda5050::OperatingMode)>::value,
        "Expected type vda5050::OperatingMode(vda5050::OperatingMode)");
    std::unique_lock lock(this->mutex_);
    auto before = this->operating_mode_;
    this->operating_mode_ = alter_function(this->operating_mode_);
    return before != this->operating_mode_;
  }

  /// \brief Alter the battery state
  /// \tparam FunctionT callable void(vda5050::BatteryState &)
  /// \param alter_function
  template <typename FunctionT> void alterBatteryState(FunctionT alter_function) {
    static_assert(
        vda5050pp::core::common::is_signature<FunctionT, void(vda5050::BatteryState &)>::value,
        "Expected type void(vda5050::BatteryState &)");
    std::unique_lock lock(this->mutex_);
    alter_function(this->battery_state_);
  }

  /// \brief Alter the safety state
  /// \tparam FunctionT callable void(vda5050::SafetyState &)
  /// \param alter_function
  template <typename FunctionT> bool alterSafetyState(FunctionT alter_function) {
    static_assert(
        vda5050pp::core::common::is_signature<FunctionT, void(vda5050::SafetyState &)>::value,
        "Expected type void(vda5050::SafetyState &)");
    std::unique_lock lock(this->mutex_);
    auto before = this->safety_state_;
    alter_function(this->safety_state_);
    return before != this->safety_state_;
  }

  /// \brief Alter the errors vector
  /// \tparam FunctionT callable void(std::vector<vda5050::Error> &)
  /// \param alter_function the alter function
  template <typename FunctionT> bool alterErrors(FunctionT alter_function) {
    static_assert(vda5050pp::core::common::is_signature<FunctionT,
                                                        void(std::vector<vda5050::Error> &)>::value,
                  "Expected type void(std::vector<vda5050::Error> &)");
    std::unique_lock lock(this->mutex_);
    alter_function(this->errors_);
    return true;
  }

  /// \brief Alter the infos vector
  /// \tparam FunctionT callable void(std::vector<vda5050::Info> &)
  /// \param alter_function
  template <typename FunctionT> void alterInfos(FunctionT alter_function) {
    static_assert(
        vda5050pp::core::common::is_signature<FunctionT, void(std::vector<vda5050::Info> &)>::value,
        "Expected type void(std::vector<vda5050::Info> &)");
    std::unique_lock lock(this->mutex_);
    alter_function(this->information_);
  }

  ///
  ///\brief Dump all data to a vda5050::State
  ///
  ///\param state the state to write to
  ///
  void dumpTo(vda5050::State &state);
};

}  // namespace vda5050pp::core::state

#endif  // VDA5050_2B_2B_CORE_STATE_STATUS_MANAGER_H_
