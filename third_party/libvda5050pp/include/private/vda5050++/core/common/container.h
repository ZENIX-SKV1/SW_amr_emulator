//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_COMMON_CONTAINER_H_
#define VDA5050_2B_2B_CORE_COMMON_CONTAINER_H_

#include <algorithm>

namespace vda5050pp::core::common {

///
///\brief Check if a container contains a specific element (equality based)
///
///\tparam ContainerT the container type
///\tparam ValueT the type of the value inside of the container
///\param container the container
///\param value the value to search
///\return true iff the container contains the value
///
template <typename ContainerT, typename ValueT>
constexpr bool contains(const ContainerT &container, const ValueT &value) noexcept(true) {
  return std::find(cbegin(container), cend(container), value) != cend(container);
}

}  // namespace vda5050pp::core::common

#endif  // VDA5050_2B_2B_CORE_COMMON_CONTAINER_H_
