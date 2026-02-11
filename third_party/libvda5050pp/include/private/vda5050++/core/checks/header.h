//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_CHECKS_HEADER_H_
#define VDA5050_2B_2B_CORE_CHECKS_HEADER_H_

#include <vda5050/Error.h>
#include <vda5050/Header_vda5050.h>

#include <list>

namespace vda5050pp::core::checks {

///
///\brief Check validity of a header against the AGVDescription
///
///\param header the header to check
///\return std::list<vda5050::Error>  a list of validation errors, if empty the header is valid
///
std::list<vda5050::Error> checkHeader(const vda5050::HeaderVDA5050 &header);

}  // namespace vda5050pp::core::checks

#endif  // VDA5050_2B_2B_CORE_CHECKS_HEADER_H_
