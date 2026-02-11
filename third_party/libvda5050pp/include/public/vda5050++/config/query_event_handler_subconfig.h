// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// This file contains the QueryEventHandlerSubConfig class.
//
//

#ifndef PUBLIC_VDA5050_2B_2B_CONFIG_QUERY_EVENT_HANDLER_SUBCONFIG_H_
#define PUBLIC_VDA5050_2B_2B_CONFIG_QUERY_EVENT_HANDLER_SUBCONFIG_H_

#include <optional>
#include <set>
#include <string>

#include "vda5050++/config/module_subconfig.h"

namespace vda5050pp::config {

///
///\brief The QueryEventHandler sub config.
///
/// Contains:
/// - default pauseable/resumable
/// - default accept zone set behaviour
/// - deviation based "node trivially reachable" check
///
///
class QueryEventHandlerSubConfig : public vda5050pp::config::ModuleSubConfig {
private:
  std::optional<bool> default_pauseable_success_;
  std::optional<bool> default_resumable_success_;
  std::optional<bool> default_accept_zone_set_success_;
  std::optional<std::set<std::string, std::less<>>> default_accept_zone_sets_;
  std::optional<bool> allow_node_trivially_reachable_deviation_check_;
  std::optional<double> reachable_check_default_node_deviation_xy_;
  std::optional<double> reachable_check_default_node_deviation_theta_;
  std::optional<double> reachable_check_overwrite_node_deviation_xy_;
  std::optional<double> reachable_check_overwrite_node_deviation_theta_;

protected:
  ///
  ///\brief Read all module settings from the ConfigNode.
  ///
  ///\param node the ConfigNode to read from
  ///
  void getFrom(const ConstConfigNode &node) override;

  ///
  ///\brief Write all module setting sto the ConfigNode.
  ///
  ///\param node the ConfigNode to write to
  ///
  void putTo(ConfigNode &node) const override;

public:
  ///
  ///\brief Set the default PauseableSuccess value. If no handler is set, this determines the
  /// answer.
  ///
  ///\param value std::nullopt if there is no default, otherwise use the given default success.
  ///
  void setDefaultPauseableSuccess(std::optional<bool> value);

  ///
  ///\brief Get the default PauseableSuccess value.
  ///
  ///\return std::optional<bool>
  ///
  std::optional<bool> getDefaultPauseableSuccess() const;

  ///
  ///\brief Set the default ResumableSuccess value. If no handler is set, this determines the
  /// answer.
  ///
  ///\param value std::nullopt if there is no default, otherwise use the given default success.
  ///
  void setDefaultResumableSuccess(std::optional<bool> value);

  ///
  ///\brief Get the default ResumableSuccess value.
  ///
  ///\return std::optional<bool>
  ///
  std::optional<bool> getDefaultResumableSuccess() const;

  ///
  ///\brief Set the default AcceptZoneSet value. If no handler is set, this determines the answer.
  ///
  ///\param value std::nullopt if there is no default, otherwise use the given default success.
  ///
  void setDefaultAcceptZoneSetSuccess(std::optional<bool> value);

  ///
  ///\brief Get the default AcceptZoneSetSuccess value.
  ///
  ///\return std::optional<bool>
  ///
  std::optional<bool> getDefaultAcceptZoneSetSuccess() const;

  ///
  ///\brief Set the a list if allowed zone-sets to check against, if ther is no handler answering
  /// this query.
  ///
  ///\param value std::nullopt if this check shall not be used. Otherwise check against the set.
  ///
  void setDefaultAcceptZoneSets(const std::set<std::string, std::less<>> &value);

  ///
  ///\brief Get the default AcceptZoneSets.
  ///
  ///\return const std::optional<std::set<std::string>>&
  ///
  const std::optional<std::set<std::string, std::less<>>> &getDefaultAcceptZoneSets() const;

  ///
  ///\brief Set the allow flag for the node trivially reachable deviation check.
  /// If the flag is false, the deviation will not be used for the check.
  /// If the flag is true or nullopt, the deviation will be used for the check.
  ///
  ///\param value the new value for the flag.
  ///
  void setAllowNodeTriviallyReachableDeviationCheck(std::optional<bool> value);

  ///
  ///\brief Get the allow flag for the node trivially reachable deviation check.
  ///
  ///\return std::optional<bool> the flag (if set)
  ///
  std::optional<bool> getAllowNodeTriviallyReachableDeviationCheck() const;

  ///
  ///\brief Set the default deviation range use to check if a node is trivially reachable.
  /// In case the node has no deviation set, this value will be used.
  ///
  ///\param value the new value for the deviation range. (std::nullopt for no default (implicit 0))
  ///
  void setReachableCheckDefaultNodeDeviationXY(std::optional<double> value);

  ///
  ///\brief Get the default deviation range use to check if a node is trivially reachable.
  ///
  ///\return std::optional<double> the deviation range.
  ///
  std::optional<double> getReachableCheckDefaultNodeDeviationXY() const;

  ///
  ///\brief Set the default deviation theta range use to check if a node is trivially reachable.
  /// In case the node has no deviation set, this value will be used.
  ///
  ///\param value the new value for the deviation theta range. (std::nullopt for no default
  ///(implicit 0))
  ///
  void setReachableCheckDefaultNodeDeviationTheta(std::optional<double> value);

  ///
  ///\brief Get the default deviation theta range use to check if a node is trivially reachable.
  ///
  ///\return std::optional<double> the deviation theta range.
  ///
  std::optional<double> getReachableCheckDefaultNodeDeviationTheta() const;

  ///
  ///\brief Set the deviation range use to check if a node is trivially reachable. Used for ALL
  /// nodes.
  ///
  ///\param value the new value for the deviation range. (std::nullopt for no override)
  ///
  void setReachableCheckOverwriteNodeDeviationXY(std::optional<double> value);

  ///
  ///\brief Get the deviation range use to check if a node is trivially reachable. Used for ALL
  /// nodes.
  ///
  ///\return std::optional<double> the deviation range.
  ///
  std::optional<double> getReachableCheckOverwriteNodeDeviationXY() const;

  ///
  ///\brief Set the deviation theta range use to check if a node is trivially reachable. Used for
  /// ALL nodes.
  ///
  ///\param value  the new value for the deviation theta range. (std::nullopt for no override)
  ///
  void setReachableCheckOverwriteNodeDeviationTheta(std::optional<double> value);

  ///
  ///\brief Get the deviation theta range use to check if a node is trivially reachable. Used for
  /// ALL nodes.
  ///
  ///\return std::optional<double> the deviation theta range.
  ///
  std::optional<double> getReachableCheckOverwriteNodeDeviationTheta() const;
};

}  // namespace vda5050pp::config

#endif  // PUBLIC_VDA5050_2B_2B_CONFIG_QUERY_EVENT_HANDLER_SUBCONFIG_H_
