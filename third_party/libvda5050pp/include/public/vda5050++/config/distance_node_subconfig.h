// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3
//
// TODO: Short description

#ifndef PUBLIC_VDA5050_2B_2B_CONFIG_DISTANCE_NODE_SUBCONFIG_H_
#define PUBLIC_VDA5050_2B_2B_CONFIG_DISTANCE_NODE_SUBCONFIG_H_

#include <optional>

#include "vda5050++/config/module_subconfig.h"

namespace vda5050pp::config {

///
///\brief The NodeReachedHandler's SubConfig.
///
/// Contains:
///   - Default node deviations to use, if they were not given by the order.
///   - Overwrite node deviations, which will always be used.
///
class DistanceNodeSubConfig : public ModuleSubConfig {
public:
  enum InterpolationType { NONE = 0, LINEAR };

private:
  std::optional<InterpolationType> interpolation_type_;

protected:
  ///
  ///\brief Read the values from a ConfigNode.
  ///
  ///\param node the ConfigNode to read from.
  ///
  void getFrom(const ConstConfigNode &node) override;

  ///
  ///\brief Write the values to a ConfigNode.
  ///
  ///\param node the ConfigNode to write to.
  ///
  void putTo(ConfigNode &node) const override;

public:
  ///
  ///\brief Set the interpolation type.
  ///
  ///\param new_value new interpolation type.
  ///
  void setInterpolationType(std::optional<InterpolationType> new_value);

  ///
  ///\brief Get the interpolation type.
  ///
  ///\return std::optional<InterpolationType>
  ///
  std::optional<InterpolationType> getInterpolationType() const;
};

}  // namespace vda5050pp::config

#endif  // PUBLIC_VDA5050_2B_2B_CONFIG_NODE_REACHED_SUBCONFIG_H_