//  Copyright Open Logistics Foundation
//
//  Licensed under the Open Logistics Foundation License 1.3.
//  For details on the licensing terms, see the LICENSE file.
//  SPDX-License-Identifier: OLFL-1.3
//

#ifndef VDA5050_2B_2B_CORE_CHECKS_ORDER_H_
#define VDA5050_2B_2B_CORE_CHECKS_ORDER_H_

#include <vda5050/Error.h>
#include <vda5050/Order.h>

#include <list>

namespace vda5050pp::core::checks {

///
///\brief Check the consistency of an order's graph.
///
/// - Sequence IDs must be unique.
/// - Sequence IDs must not contain skips
/// - Node/Edge alternation
/// - non-empty
/// - valid base/horizon separation
///
///\param order the order to check
///\return std::list<vda5050::Error> a list of validation errors, if empty the order is valid
///
std::list<vda5050::Error> checkOrderGraphConsistency(const vda5050::Order &order);

///
///\brief Check if the order ID/updateId is valid.
///
/// When having same order ID as current order:
/// - orderUpdateIDs may only increase or stay equal
///
///\param order
///\return std::list<vda5050::Error>  a list of validation errors, if empty the order is valid
///
std::list<vda5050::Error> checkOrderId(const vda5050::Order &order);

///
///\brief Check if the order appends to the current order.
///
/// - orderUpdateIDs must be increasing when orderId is equal
/// - orders must properly stitch together (last-base-node = first-new-base-node)
///
///\param order the order to check
///\return std::list<vda5050::Error> a list of validation errors, if empty the order is valid
///
std::list<vda5050::Error> checkOrderAppend(const vda5050::Order &order);

///
///\brief Check of the actionIds of the order are unique.
///
///\param order the order to check
///\return std::list<vda5050::Error> a list of validation errors, if empty the order is valid
///
std::list<vda5050::Error> checkOrderActionIds(const vda5050::Order &order);

}  // namespace vda5050pp::core::checks

#endif  // VDA5050_2B_2B_CORE_CHECKS_ORDER_H_
