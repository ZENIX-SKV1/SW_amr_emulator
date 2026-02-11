//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_CONFIG_H_
#define PRIVATE_VDA5050_2B_2B_CORE_CONFIG_H_

#include <spdlog/fmt/fmt.h>
#include <toml++/toml.h>

#include "vda5050++/config.h"
#include "vda5050++/core/common/exception.h"

namespace vda5050pp::core::config {

///
///\brief The ConfigNode wraps a toml::node and provides a hides the dependency behind the
/// vda5050pp::config::ConfigNode interface.
///
/// It's a read/write reference.
///
class ConfigNode : public vda5050pp::config::ConfigNode {
private:
  ///\brief the actual node
  toml::node *node_;

public:
  ///
  ///\brief Upcast a vda5050pp::config::ConfigNode to a ConfigNode to access the toml::node.
  ///
  ///\param node the opaque node
  ///\return ConfigNode& the transparent node
  ///
  static inline ConfigNode &upcast(vda5050pp::config::ConfigNode &node) {
    return static_cast<ConfigNode &>(node);
  }

  ///
  ///\brief Upcast a vda5050pp::config::ConfigNode to a ConfigNode to access the toml::node.
  ///
  ///\param node the opaque node
  ///\return const ConfigNode& the transparent node
  ///
  static inline const ConfigNode &upcast(const vda5050pp::config::ConfigNode &node) {
    return static_cast<const ConfigNode &>(node);
  }

  ///
  ///\brief Construct a ConfigNode from a toml::node
  ///
  ///\param node the toml node
  ///
  explicit ConfigNode(toml::node &node) : node_(&node) {}

  ///
  ///\brief Construct a ConfigNode from a toml::node
  ///
  ///\param node the toml node pointer
  ///
  explicit ConfigNode(toml::node *node) : node_(node) {}

  ///
  ///\brief Get a node view to the wrapped toml::node
  ///
  ///\return toml::node_view<toml::node> the node view
  ///
  toml::node_view<toml::node> get();

  ///
  ///\brief Get a node view to the wrapped toml::node
  ///
  ///\return toml::node_view<const toml::node> the node view
  ///
  toml::node_view<const toml::node> get() const;
};

///
///\brief The ConstConfigNode wraps a toml::node and provides a hides the dependency behind the
/// vda5050pp::config::ConstConfigNode interface.
///
/// It's a read-only reference.
///
class ConstConfigNode : public vda5050pp::config::ConstConfigNode {
private:
  ///\brief the actual node
  const toml::node *node_;

public:
  ///
  ///\brief Upcast a vda5050pp::config::ConstConfigNode to a ConstConfigNode to access the
  /// toml::node.
  ///
  ///\param node the opaque node
  ///\return ConstConfigNode& the transparent node
  ///
  static inline ConstConfigNode &upcast(vda5050pp::config::ConstConfigNode &node) {
    return static_cast<ConstConfigNode &>(node);
  }

  ///
  ///\brief Upcast a vda5050pp::config::ConstConfigNode to a ConstConfigNode to access the
  /// toml::node.
  ///
  ///\param node the opaque node
  ///\return const ConstConfigNode& the transparent node
  ///
  static inline const ConstConfigNode &upcast(const vda5050pp::config::ConstConfigNode &node) {
    return static_cast<const ConstConfigNode &>(node);
  }

  ///
  ///\brief Construct a ConstConfigNode from a toml::node
  ///
  ///\param node the toml node
  ///
  explicit ConstConfigNode(const toml::node &node) : node_(&node) {}

  ///
  ///\brief Construct a ConstConfigNode from a toml::node
  ///
  ///\param node the toml node pointer
  ///
  explicit ConstConfigNode(const toml::node *node) : node_(node) {}

  ///
  ///\brief Get a node view to the wrapped toml::node
  ///
  ///\return toml::node_view<const toml::node> the node view
  ///
  toml::node_view<const toml::node> get() const;
};

}  // namespace vda5050pp::core::config

#endif  // PRIVATE_VDA5050_2B_2B_CORE_CONFIG_H_
