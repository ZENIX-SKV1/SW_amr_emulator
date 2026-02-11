//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef PRIVATE_VDA5050_2B_2B_CORE_MODULE_H_
#define PRIVATE_VDA5050_2B_2B_CORE_MODULE_H_

#include <string_view>

#include "vda5050++/config/types.h"
#include "vda5050++/core/instance.h"

namespace vda5050pp::core {

///
///\brief The base-class for all Modules. A Module is a part of the library that can be
/// initialized and deinitialized together with the library.
///
class Module {
public:
  virtual ~Module() = default;
  ///
  ///\brief Initialize the module.
  ///
  ///\param instance the current library instance
  ///
  virtual void initialize(vda5050pp::core::Instance &instance) = 0;

  ///
  ///\brief Deinitialize the module.
  ///
  ///\param instance the current library instance
  ///
  virtual void deinitialize(vda5050pp::core::Instance &instance) = 0;

  ///
  ///\brief Get a brief description of the module (possibly deprecated)
  ///
  ///\return std::string_view the module's description
  ///
  virtual std::string_view describe() const = 0;

  ///
  ///\brief Generate a sub-config for the module.
  ///
  ///\return std::shared_ptr<vda5050pp::config::ModuleSubConfig> the module's sub-config
  ///
  virtual std::shared_ptr<vda5050pp::config::ModuleSubConfig> generateSubConfig() const {
    return std::make_shared<vda5050pp::config::ModuleSubConfig>();
  }
};

///
///\brief A convenience 0-size class for static registration of modules.
/// When declaring a static object of this type, the module is registered with the library
/// statically.
///
/// IMPORTANT: This should only be used in the same compilation unit, as the module registry,
/// i.e. instance.cpp. Otherwise the module is possibly registered before the registry exists.
///
///
///\tparam M the concrete module type to construct and register
///\tparam &key the key string under which the module is registered.
///
template <typename M, auto &key> struct AutoRegisterModule {
  AutoRegisterModule() { Instance::registerModule(key, std::make_shared<M>()); }
};

}  // namespace vda5050pp::core

#endif  // PRIVATE_VDA5050_2B_2B_CORE_MODULE_H_
